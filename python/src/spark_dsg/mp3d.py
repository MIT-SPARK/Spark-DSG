# Copyright 2022, Massachusetts Institute of Technology.
# All Rights Reserved
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Research was sponsored by the United States Air Force Research Laboratory and
# the United States Air Force Artificial Intelligence Accelerator and was
# accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
# and conclusions contained in this document are those of the authors and should
# not be interpreted as representing the official policies, either expressed or
# implied, of the United States Air Force or the U.S. Government. The U.S.
# Government is authorized to reproduce and distribute reprints for Government
# purposes notwithstanding any copyright notation herein.
#
#
"""Functions for parsing a house file."""
from spark_dsg._dsg_bindings import (
    DsgLayers,
    NodeSymbol,
    RoomNodeAttributes,
)
import seaborn as sns
import numpy as np
import shapely.geometry
import shapely.ops
import heapq


def _filter_line(line):
    contents = line.strip().split(" ")[1:]
    return line[0], [x for x in contents if x != ""]


def _make_vec3(raw_list, start):
    return np.array([float(x) for x in raw_list[start : start + 3]])


def _parse_region(line):
    """Get region info from a line."""
    return {
        "region_index": int(line[0]),
        "level_index": int(line[1]),
        "label": line[4],
        "pos": _make_vec3(line, 5),
        "bbox_min": _make_vec3(line, 8),
        "bbox_max": _make_vec3(line, 11),
        "height": float(line[14]),
    }


def _parse_surface(line):
    """Get surface info from a line."""
    return {
        "surface_index": int(line[0]),
        "region_index": int(line[1]),
        "label": line[3],
        "pos": _make_vec3(line, 4),
        "normal": _make_vec3(line, 7),
        "bbox_min": _make_vec3(line, 10),
        "bbox_max": _make_vec3(line, 13),
    }


def _parse_vertex(line):
    """Get vertex info from a line."""
    return {
        "vertex_index": int(line[0]),
        "surface_index": int(line[1]),
        "label": line[2],
        "pos": _make_vec3(line, 3),
        "normal": _make_vec3(line, 6),
    }


def _parse_category(line):
    """Get category info from a line."""
    return {
        "category_index": int(line[0]),
        "category_mapping_index": int(line[1]),
        "category_mapping_name": line[2],
        "mpcat40_index": int(line[3]),
        "mpcat40_name": line[4],
    }


def _parse_object(line):
    """Get object info from a line."""
    return {
        "object_index": int(line[0]),
        "region_index": int(line[1]),
        "category_index": int(line[2]),
        "pos": _make_vec3(line, 3),
        "a0": _make_vec3(line, 6),
        "a1": _make_vec3(line, 9),
        "r": _make_vec3(line, 12),
    }


def _parse_segment(line):
    """Get segment info from a line."""
    return {
        "segment_index": int(line[0]),
        "object_index": int(line[1]),
        "id": int(line[2]),
    }


def _assign_room_edges(G):
    for place in G.get_layer(DsgLayers.PLACES).nodes:
        curr_room = place.get_parent()
        if curr_room is None:
            continue

        # check neighboring place node for room connections
        neighbors = [G.get_node(i).get_parent() for i in place.siblings()]
        neighbors = set([x for x in neighbors if x is not None and x != curr_room])

        for neighbor in neighbors:
            G.insert_edge(curr_room, neighbor)


PARSERS = {
    "V": _parse_vertex,
    "S": _parse_surface,
    "R": _parse_region,
    "C": _parse_category,
    "O": _parse_object,
    "E": _parse_segment,
}


class Mp3dRoom:
    """Class capturing arbitrary room polygon and semantic label."""

    def __init__(self, index, region, vertices, angle_deg=0.0):
        """
        Construct a room.

        Args:
            index (int): room index (relative to house file)
            region (Dict[str, Any]): region information for specific room
            vertices (List[Iterable[float]]): 3D corners of the bounding polygon
            angle_deg (float): Angle to roate polygon by
        """
        self._index = index
        self._label = region["label"]
        self._pos = np.array(
            [
                region["pos"][0],
                region["pos"][1],
                region["pos"][2] + region["height"] / 2.0,
            ]
        )

        self._min_z = region["bbox_min"][2]
        self._max_z = region["bbox_max"][2]

        # house files are rotated 90 degreees from Hydra convention
        xy_polygon = shapely.geometry.Polygon([x[:2].tolist() for x in vertices])

        theta = np.deg2rad(angle_deg)
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

        self._pos[:2] = R @ self._pos[:2]
        rotated_vertices = [
            (R @ np.array(x)).tolist() for x in xy_polygon.exterior.coords
        ]
        self._polygon_xy = shapely.geometry.Polygon(rotated_vertices)
        while not self._polygon_xy.is_valid:  # a few rooms have invalid xy-polygon
            rotated_vertices = rotated_vertices[:-1]
            self._polygon_xy = shapely.geometry.Polygon(rotated_vertices)

    def pos_on_same_floor(self, pos):
        """
        Check if a 3d position is within z-axis bounds.

        Args:
            pos (List[float]): 3d position to check

        Returns:
            bool: True if position falls inside [min_z, max_z]
        """
        return self._min_z <= pos[2] <= self._max_z

    def pos_inside_room(self, pos):
        """
        Check if a 3d position falls within the bounds of the room.

        Args:
            pos (List[float]): 3d position to check

        Returns:
            bool: True if position falls inside [min_z, max_z] and polygon bounds
        """
        if not self.pos_on_same_floor(pos):
            return False

        xy_pos = shapely.geometry.Point(pos[0], pos[1])
        return self._polygon_xy.contains(xy_pos)

    def get_polygon_xy(self):
        """Get the xy bounding polygon of the room."""
        return self._polygon_xy

    def get_id(self):
        """Get a NodeSymbol using the original region index from the mp3d file."""
        return NodeSymbol("R", self._index)

    def get_attrs(self, color):
        """Get suitable room node attributes for inclusion in a scene graph."""
        attrs = RoomNodeAttributes()
        attrs.color = color
        attrs.name = str(NodeSymbol("R", self._index))
        attrs.position = self._pos
        attrs.last_update_time_ns = 0
        attrs.semantic_label = ord(self._label)
        return attrs

    @property
    def centroid(self):
        """Get room centroid."""
        return self._pos

    @property
    def semantic_label(self):
        """
        Get the semantic label as a uint8_t.

        The returned semantic label is the ascii value of the original character used to
        represent the mp3d category, i.e., that 'a' (bathroom) maps to 97.
        """
        return ord(self._label)


def load_mp3d_info(house_path):
    """
    Load room info from a GT house file.

    Args:
        house_path (str): Path to house file

    Returns:
        Dict[str, List[Dict[Str, Any]]]): Parsed house file information
    """
    info = {x: [] for x in PARSERS}
    with open(house_path, "r") as fin:
        for line in fin:
            line_type, line = _filter_line(line)
            if line_type not in PARSERS:
                continue

            new_element = PARSERS[line_type](line)
            info[line_type].append(new_element)

    return info


def get_rooms_from_mp3d_info(mp3d_info, angle_deg=-90.0):
    """
    Generate a list of Mp3dRoom objects from ground-truth segmentation.

    Args:
        mp3d_info (Dict[str, List[Dict[Str, Any]]]): Parsed house file information
        angle_deg (bool): Angle to rotate rooms by

    Returns:
        List[Mp3dRoom]: Room information from mp3d house file
    """
    rooms = []
    for region in mp3d_info["R"]:
        r_index = region["region_index"]

        valid_surfaces = []
        for surface in mp3d_info["S"]:
            if surface["region_index"] == r_index:
                valid_surfaces.append(surface["surface_index"])

        vertices = []
        for vertex in mp3d_info["V"]:
            if vertex["surface_index"] in valid_surfaces:
                vertices.append(vertex["pos"])

        rooms.append(Mp3dRoom(r_index, region, vertices, angle_deg=angle_deg))

    return rooms


def _expand_node(frontier, G, node, parent):
    for sibling in node.siblings():
        sibling_node = G.get_node(sibling)
        if sibling_node.has_parent():
            continue

        edge = G.get_edge(node.id.value, sibling)
        # we want to prioritize expansion by the maximum distance to an obstacle and
        # heapq is smallest first, so we invert the distance
        heapq.heappush(frontier, (-edge.info.weight, sibling, parent))


def _get_combined_polygon(nodes, tolerance=0.05):
    points = []
    for node in nodes:
        points.append(
            shapely.geometry.Point(
                node.attributes.position[0], node.attributes.position[1]
            ).buffer(node.attributes.distance)
        )
    return shapely.ops.unary_union(points).simplify(tolerance, preserve_topology=False)


def expand_rooms(G_old, verbose=False):
    """
    Attempt to label any unassigned place nodes.

    Assigns place nodes to rooms via floodfill with a frontier initialized from places
    nodes already assigned by rooms, prioritizing expansion by distance to the nearest
    obstacle. Also attempts to add more room-to-room edges.

    Args:
        G_old (DynamicSceneGraph): Input graph
        verbose (bool): Whether or not to print resulting statistics

    Returns:
        DynamicSceneGraph: A copy of G_old wih more place nodes assigned to rooms
    """
    G = G_old.clone()
    places = G.get_layer(DsgLayers.PLACES)

    unlabeled = set([])
    frontier = []
    for node in places.nodes:
        parent = node.get_parent()
        if parent is None:
            unlabeled.add(node.id.value)
            continue

        _expand_node(frontier, G, node, parent)

    visited = set([])
    while len(frontier) > 0:
        _, node_id, room = heapq.heappop(frontier)
        if node_id in visited:
            continue

        G.insert_edge(node_id, room)
        visited.add(node_id)

        node = G.get_node(node_id)
        _expand_node(frontier, G, node, room)

    prev_edges = G.get_layer(DsgLayers.ROOMS).num_edges()
    _assign_room_edges(G)
    new_edges = G.get_layer(DsgLayers.ROOMS).num_edges()

    if verbose:
        remaining = unlabeled - visited
        verbose_str = ""
        verbose_str += f"Places: {places.num_nodes()}"
        verbose_str += f" (Prev Unassigned: {len(unlabeled)}"
        verbose_str += f", Assigned: {len(visited)}, Unassigned: {len(remaining)})"
        verbose_str += f", Room edges: {prev_edges} (prev) -> {new_edges} (expanded)"
        print(verbose_str)

    return G


def repartition_rooms(
    G_prev,
    mp3d_info,
    angle_deg=-90.0,
    min_iou_threshold=0.0,
    colors=None,
    verbose=False,
):
    """
    Create a copy of the DSG with ground-truth room nodes.

    Args:
        G_prev (DynamicSceneGraph): Graph to repartition
        mp3d_info (Dict[str, List[Dict[Str, Any]]]): Parsed house file information
        angle_deg (float): Degrees to rotate the parsed rooms
        min_iou_threshold (float): Minimum intersection over union for valid rooms
        colors (Optional[List[Iterable[float]]]): Optional colormap to assign to rooms
        verbose (bool): Print information about repartition process

    Returns:
        DynamicSceneGraph: A copy of G_prev with a new room layer taken from mp3d_info
    """
    G = G_prev.clone()

    # remove existing rooms
    existing_rooms = [room.id.value for room in G.get_layer(DsgLayers.ROOMS).nodes]
    for i in existing_rooms:
        G.remove_node(i)

    new_rooms = get_rooms_from_mp3d_info(mp3d_info, angle_deg)

    if colors is None:
        colors = sns.color_palette("husl", len(new_rooms))

    buildings = [node.id.value for node in G.get_layer(DsgLayers.BUILDINGS).nodes]

    # add new room nodes and connect them to building node
    room_id_to_gt_index = dict()  # map room node id in dsg to index in new_rooms
    for index, room in enumerate(new_rooms):
        color = np.array([int(255 * c) for c in colors[index % len(colors)]][:3])
        attrs = room.get_attrs(color)

        room_id = room.get_id()
        G.add_node(DsgLayers.ROOMS, room_id.value, attrs)
        if len(buildings) > 0:
            G.insert_edge(room_id.value, buildings[0])

        room_id_to_gt_index[room_id.value] = index

    # connect new room nodes to places nodes
    missing_nodes = []
    for place in G.get_layer(DsgLayers.PLACES).nodes:
        pos = G.get_position(place.id.value)

        for room in new_rooms:
            if not room.pos_inside_room(pos):
                continue

            room_id = room.get_id()
            G.insert_edge(place.id.value, room_id.value)
            break  # avoid labeling node as missing
        else:
            missing_nodes.append(place)
    if verbose:
        print(f"Found {len(missing_nodes)} places node outside of room segmentations.")

    # add room to room edges
    _assign_room_edges(G)

    # filter out rooms that have no children or less than
    # the specified IoU with ground truth
    invalid_rooms = []
    for room in G.get_layer(DsgLayers.ROOMS).nodes:
        places_nodes = [G.get_node(i) for i in room.children()]
        if len(places_nodes) == 0:
            invalid_rooms.append(room.id.value)
            continue

        if min_iou_threshold > 0:
            explored_polygon = _get_combined_polygon(places_nodes)
            xy_polygon = new_rooms[room_id_to_gt_index[room.id.value]].get_polygon_xy()
            intersection_area = xy_polygon.intersection(explored_polygon).area
            union_area = xy_polygon.union(explored_polygon).area
            if verbose:
                debug_str = ", ".join(
                    [
                        f"explore area: {explored_polygon.area}",
                        f"gt area: {xy_polygon.area}",
                        f"intersection_area: {intersection_area}",
                        f"union_area: {union_area}",
                    ]
                )
                print(debug_str)
            if intersection_area / union_area < min_iou_threshold:
                invalid_rooms.append(room.id.value)

    for node_id in invalid_rooms:
        G.remove_node(node_id)

    if verbose:
        print(f"Removed {len(invalid_rooms)} mp3d rooms without children.")

    return G


def add_gt_room_label(
    G,
    mp3d_info,
    min_iou_threshold=0.01,
    angle_deg=-90.0,
    use_hydra_polygon=False,
    verbose=False,
):
    """
    Add ground-truth room label to DSG based on maximum area of intersection.

    Args:
        G (DynamicSceneGraph): Graph to add labels to
        mp3d_info (Dict[str, List[Dict[Str, Any]]]): Parsed house file information
        min_iou_threshold (float): Minimum intersection over union for rooms to overlap
        use_hydra_polygon (bool): Use places node polygon instead of rectangle
        angle_deg (float): Angle to rotate mp3d rooms by
        verbose (bool): Print information about labeling process
    """
    mp3d_rooms = get_rooms_from_mp3d_info(mp3d_info, angle_deg)

    for room in G.get_layer(DsgLayers.ROOMS).nodes:
        if verbose:
            print("DSG", room.id, "to ground-truth room - area of intersection")

        if use_hydra_polygon:
            places_nodes = [G.get_node(i) for i in room.children()]
            hydra_polygon = _get_combined_polygon(places_nodes)
        else:
            bbox = room.attributes.bounding_box
            bbox_min = bbox.min_corner()
            bbox_max = bbox.max_corner()
            hydra_polygon = shapely.geometry.Polygon(
                [
                    [bbox_max[0], bbox_min[1]],
                    [bbox_min[0], bbox_max[1]],
                    [bbox_max[0], bbox_max[1]],
                    [bbox_max[0], bbox_min[1]],
                ]
            )

        # Find the ground-truth room area of intersection
        intersection_over_union = []
        for idx, mp3d_room in enumerate(mp3d_rooms):
            xy_polygon = mp3d_room.get_polygon_xy()
            # check whether hydra room position is inside the ground-truth room
            if not mp3d_room.pos_on_same_floor(room.attributes.position):
                continue
            intersection_area = xy_polygon.intersection(hydra_polygon).area
            union_area = xy_polygon.union(hydra_polygon).area
            intersection_over_union.append((idx, intersection_area / union_area))
            if verbose:
                curr_iou = intersection_over_union[-1][1]
                iou_str = f"({curr_iou:.2f} {intersection_area:.2f} / {union_area:.2f})"
                print(f"  {mp3d_room.get_id()} - {iou_str}")

        max_index, max_iou = max(intersection_over_union, key=lambda x: x[1])
        if max_iou < min_iou_threshold:
            continue

        # Update DSG room label
        gt_room = mp3d_rooms[max_index]
        gt_label = gt_room.semantic_label
        room.attributes.semantic_label = gt_label
        if verbose:
            print(f"  {room.id} <- {gt_room.get_id()} (label: {chr(gt_label)})")
