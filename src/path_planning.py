from __future__ import annotations
import math
from typing import List
from src.models import CarPose, Cone, Path2D

class PathPlanning:

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:

        cx = self.car_pose.x
        cy = self.car_pose.y
        yaw = self.car_pose.yaw

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        blue = []
        yellow = []

        # Convert cones to the car frame
        for cone in self.cones:

            dx = cone.x - cx
            dy = cone.y - cy

            forward = dx * cos_yaw + dy * sin_yaw
            side = -dx * sin_yaw + dy * cos_yaw

            data = (cone.x, cone.y, forward, side)

            if cone.color == 1:
                blue.append(data)
            else:
                yellow.append(data)

        blue.sort(key=lambda p: p[2])
        yellow.sort(key=lambda p: p[2])

        gates = []
        extension_x = cos_yaw
        extension_y = sin_yaw

        # ==========================
        # Both sides have cones
        # ==========================
        if blue and yellow:
            #=============================================
            # Same number of cones:
            # simply take the middle point between them
            #=============================================

            if len(blue) == len(yellow):

                for b, y in zip(blue, yellow):

                    x = (b[0] + y[0]) / 2
                    y_pos = (b[1] + y[1]) / 2

                    gates.append((x, y_pos))

                if len(gates) >= 2:

                    dx = gates[-1][0] - gates[0][0]
                    dy = gates[-1][1] - gates[0][1]

                    length = math.hypot(dx, dy)

                    if length > 0:
                        extension_x = dx / length
                        extension_y = dy / length


            # One side has more cones
            else:

                if len(blue) > len(yellow):
                    dense = blue
                    sparse = yellow
                else:
                    dense = yellow
                    sparse = blue

                first = dense[0]

                dx = dense[-1][0] - first[0]
                dy = dense[-1][1] - first[1]

                length = math.hypot(dx, dy)

                if length > 0:
                    direction_x = dx / length
                    direction_y = dy / length
                else:
                    direction_x = cos_yaw
                    direction_y = sin_yaw


                # Make the direction point roughly away from the car
                if ((dense[-1][0] - cx) * direction_x +
                    (dense[-1][1] - cy) * direction_y) < 0:

                    direction_x = -direction_x
                    direction_y = -direction_y

                extension_x = direction_x
                extension_y = direction_y

                sparse_x = sparse[0][0]
                sparse_y = sparse[0][1]

                base_x = first[0]
                base_y = first[1]


                # Find where the sparse cone lies on the dense line
                t = ((sparse_x - base_x) * direction_x +
                     (sparse_y - base_y) * direction_y)

                foot_x = base_x + t * direction_x
                foot_y = base_y + t * direction_y

                for cone in dense:

                    shift = ((cone[0] - foot_x) * direction_x +
                             (cone[1] - foot_y) * direction_y)

                    virtual_x = sparse_x + shift * direction_x
                    virtual_y = sparse_y + shift * direction_y

                    middle_x = (cone[0] + virtual_x) / 2
                    middle_y = (cone[1] + virtual_y) / 2

                    gates.append((middle_x, middle_y))


        # ===========================
        # Only one side has cones
        # ===========================
        elif blue or yellow:

            side = blue if blue else yellow

            # Only one cone
            if len(side) == 1:

                cone = side[0]

                # Move one meter toward the inside of the track
                if blue:
                    # Blue is the left boundary
                    nx = sin_yaw
                    ny = -cos_yaw
                else:
                    # Yellow is the right boundary
                    nx = -sin_yaw
                    ny = cos_yaw

                gates.append((
                    cone[0] + nx,
                    cone[1] + ny
                ))

            # Two or more cones on the same side
            else:

                dx = side[-1][0] - side[0][0]
                dy = side[-1][1] - side[0][1]

                length = math.hypot(dx, dy)

                if length > 0:
                    direction_x = dx / length
                    direction_y = dy / length
                else:
                    direction_x = cos_yaw
                    direction_y = sin_yaw

                # Make the direction point away from the car
                if side[-1][2] < side[0][2]:
                    direction_x = -direction_x
                    direction_y = -direction_y

                extension_x = direction_x
                extension_y = direction_y

                # Offset the boundary toward the car
                for i, cone in enumerate(side):

                    if i == 0 or i == len(side) - 1:

                        tx = direction_x
                        ty = direction_y

                    else:

                        tx = side[i + 1][0] - side[i - 1][0]
                        ty = side[i + 1][1] - side[i - 1][1]

                        l = math.hypot(tx, ty)

                        if l > 0:
                            tx /= l
                            ty /= l

                    # Perpendicular direction
                    nx = -ty
                    ny = tx

                    # Make the normal point toward the car
                    to_car_x = cx - cone[0]
                    to_car_y = cy - cone[1]

                    if to_car_x * nx + to_car_y * ny < 0:
                        nx = -nx
                        ny = -ny

                    # Assume track width is about 2 m
                    # so center is 1 m from the boundary
                    gates.append((
                        cone[0] + nx,
                        cone[1] + ny
                    ))

        ##=================##
        # Create the path
        ##=================##

        gates.sort(
            key=lambda p: math.hypot(p[0] - cx, p[1] - cy)
        )

        # Convert gates to car frame
        points = [(0.0, 0.0)]

        for x, y in gates:

            forward = (
                (x - cx) * cos_yaw +
                (y - cy) * sin_yaw
            )

            side = (
                -(x - cx) * sin_yaw +
                (y - cy) * cos_yaw
            )

            if math.hypot(
                forward - points[-1][0],
                side - points[-1][1]
            ) > 0.15:

                points.append((forward, side))

        # No cones
        if len(points) == 1:

            points.append((6.0, 0.0))

        else:

            total_length = 0.0

            for i in range(1, len(points)):
                total_length += math.hypot(
                    points[i][0] - points[i - 1][0],
                    points[i][1] - points[i - 1][1]
                )

            # Make the path at least 6 meters
            if total_length < 6.0:

                extra = 6.0 - total_length

                forward_dir = (
                    extension_x * cos_yaw +
                    extension_y * sin_yaw
                )

                side_dir = (
                    -extension_x * sin_yaw +
                    extension_y * cos_yaw
                )

                last_forward = points[-1][0]
                last_side = points[-1][1]

                points.append((
                    last_forward + forward_dir * extra,
                    last_side + side_dir * extra
                ))

        # =====================
        # Interpolate points
        # Step size <= 0.5 m
        # =====================


        path = []

        for i in range(len(points) - 1):

            x1, y1 = points[i]
            x2, y2 = points[i + 1]

            distance = math.hypot(
                x2 - x1,
                y2 - y1
            )

            number = max(
                1,
                math.ceil(distance / 0.35)
            )

            for j in range(number):

                t = j / number

                x = x1 + t * (x2 - x1)
                y = y1 + t * (y2 - y1)

                if not path or math.hypot(
                    x - path[-1][0],
                    y - path[-1][1]
                ) > 0.00001:

                    path.append((x, y))

        path.append(points[-1])

        # Convert back to world coordinates
        world_path = []

        for forward, side in path:

            x = (
                cx +
                forward * cos_yaw -
                side * sin_yaw
            )

            y = (
                cy +
                forward * sin_yaw +
                side * cos_yaw
            )

            world_path.append((x, y))

        return world_path