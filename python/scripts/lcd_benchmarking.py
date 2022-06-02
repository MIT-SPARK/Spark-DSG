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
#!/usr/bin/env python
"""Benchmarking script for LCD detection."""
import teaserpp_python
import kimera_dsg_python_bindings as kimera_dsg
import matplotlib.pyplot as plt
import kimera_dsg_lcd as lcd
import seaborn as sns
import networkx as nx
import numpy as np
import argparse
import os


class ObjectGraph:
    """Object struct."""

    def __init__(
        self,
        dsg,
        distance_threshold,
        descriptor_size,
        descriptor_length=4,
    ):
        """Make a graph."""
        object_layer = dsg.get_layer(kimera_dsg.KimeraDsgLayers.OBJECTS)

        self.ids = []
        self.labels = []
        self.node_id_mapping = {}
        self.positions = np.zeros((3, object_layer.num_nodes()))
        self.descriptor_size = descriptor_size
        self.descriptor_length = descriptor_length

        for node in object_layer.nodes:
            self.node_id_mapping[node.id] = len(self.labels)
            self.ids.append(node.id)
            self.positions[:, len(self.labels)] = node.attributes.position

            self.labels.append(node.attributes.semantic_label)

        self.make_adjacency_matrix(distance_threshold)

    def make_adjacency_matrix(self, threshold):
        """Construct the adjacency matrix for the graph."""
        matrix = np.zeros((len(self.labels), len(self.labels)))
        for i in range(self.positions.shape[1]):
            for j in range(i + 1, self.positions.shape[1]):
                distance = np.linalg.norm(
                    self.positions[0:2, i] - self.positions[0:2, j]
                )
                if distance < threshold:
                    matrix[i, j] = 1
                    matrix[j, i] = 1

        self.lcd_graph = lcd.Graph(matrix, self.labels)
        # these need to be rebuilt every time the connectivity changes
        self.descriptors = self.get_descriptors(self.ids)

    def get_descriptors(self, nodes):
        """Make descriptors for every node in the graph."""
        descriptors = []
        for node in nodes:
            descriptor = lcd.RwDescriptor(self.descriptor_size, self.descriptor_length)
            descriptor.generate(self.lcd_graph, self.node_id_mapping[node])
            descriptors.append(descriptor)

        return descriptors

    def get_nodes_near_point(self, point, radius=5.0):
        """Extract descriptors for all nodes close enough to the query point."""
        # TODO(nathan) we could consider KDtree for nearest neighbors
        to_return = []
        for index, node_id in enumerate(self.ids):
            if np.linalg.norm(point - self.positions[:, index]) < radius:
                to_return.append(node_id)

        return to_return

    def get_node_positions(self, nodes):
        """Make a 3xN matrix of positions."""
        to_return = np.zeros((3, len(nodes)))
        for index, node_id in enumerate(nodes):
            to_return[:, index] = self.positions[:, self.node_id_mapping[node_id]]

        return to_return

    def get_matches(self, descriptors, threshold=0.2, lowes_ratio=0.9):
        """Check for matches against all nodes."""
        if len(self.descriptors) < 2:
            print("Failure: Not enough object nodes in graph")
            return

        matches = {}
        match_scores = {}
        for index, descriptor in enumerate(descriptors):
            distances = np.array(
                [descriptor.match(other) for other in self.descriptors]
            )
            # find the largest two distances
            max_indices = np.argpartition(-distances, 2)
            if distances[max_indices[0]] < threshold:
                continue

            # invert distances for lowe's ratio test
            best_distance_inv = 1.0 - distances[max_indices[0]]
            second_best_distance_inv = 1.0 - distances[max_indices[1]]
            if best_distance_inv > second_best_distance_inv * lowes_ratio:
                continue

            matches[index] = self.ids[max_indices[0]]
            match_scores[index] = distances[max_indices[0]]

        best_scores = {}
        best_matches = {}
        for index, node_id in matches.items():
            if node_id in best_scores:
                if best_scores[node_id] < match_scores[index]:
                    best_matches[node_id] = index
                    best_scores[node_id] = match_scores[index]
            else:
                best_matches[node_id] = index
                best_scores[node_id] = match_scores[index]

        return {value: key for key, value in best_matches.items()}

    def solve_for_pose(self, solver, matches, source_positions):
        """Solve for a correspondence between provided and target pointclouds."""
        print("")
        print("Solving For Pose")
        print("----------------")
        matched_nodes = [node_id for _, node_id in matches.items()]
        target_positions = self.get_node_positions(matched_nodes)
        solver.solve(source_positions, target_positions)
        solution = solver.getSolution()
        target_T_source = (solution.rotation, solution.translation)
        return target_T_source

    def get_centroid(self):
        """Get geometric center of nodes."""
        return np.mean(self.positions, axis=1)

    def export_networkx(self):
        """Make a networkx graph (for drawing)."""
        G = nx.Graph()
        for idx, node_id in enumerate(self.ids):
            G.add_node(idx, node_label=node_id)

        for idx, _ in enumerate(self.ids):
            edge_list = self.lcd_graph.adjacency_matrix[idx, :]
            for other_idx, value in enumerate(edge_list):
                if value == 1.0:
                    G.add_edge(idx, other_idx)

        return G

    def draw_graph(self):
        """Show the graph using matplotlib."""
        nx_graph = self.export_networkx()
        pos_dict = {idx: self.positions[0:2, idx] for idx in range(0, len(self.ids))}
        min_label = min(self.labels)
        label_range = max(self.labels) - min_label + 1
        color_palette = sns.color_palette("husl", label_range)
        colors = [color_palette[label - min_label] for label in self.labels]

        nx.draw(nx_graph, with_labels=True, pos=pos_dict, node_color=colors)
        plt.show()


def transform_points(target_T_source, points, translation_noise=1.0e-3):
    """Transform and add noise to a set of points."""
    to_return = np.zeros(points.shape)
    source_R_target = target_T_source[0].transpose()
    source_p_target = -source_R_target.dot(target_T_source[1].reshape((3, 1)))
    for i in range(to_return.shape[1]):
        point = points[:, i].reshape((3, 1))
        new_point = source_R_target.dot(point) + source_p_target
        new_point += np.random.normal(0, translation_noise, size=(3, 1))
        to_return[:, i, np.newaxis] = new_point
    return to_return


def get_params():
    """Make some params for TEASER."""
    # Populating the parameters
    params = teaserpp_python.RobustRegistrationSolver.Params()
    params.cbar2 = 1
    params.noise_bound = 0.01
    params.estimate_scaling = False
    params.rotation_estimation_algorithm = (
        teaserpp_python.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
    )
    params.rotation_gnc_factor = 1.4
    params.rotation_max_iterations = 100
    params.rotation_cost_threshold = 1e-12
    return params


# from https://github.com/MIT-SPARK/TEASER-plusplus (examples/teaser_python_ply)
def get_angular_error(R_exp, R_est):
    """Calculate angular error."""
    return abs(
        np.arccos(min(max(((np.matmul(R_exp.T, R_est)).trace() - 1) / 2, -1.0), 1.0))
    )


def show_results(target_T_source, target_T_source_est):
    """Show some info about the results."""
    print("")
    print("Results")
    print("-------")
    print("")
    print("Ground Truth")
    print("============")
    print("Rotation:\n{}".format(target_T_source[0]))
    print("Translation: {}".format(np.squeeze(target_T_source[1])))
    print("")
    print("Estimated")
    print("=========")
    print("Rotation:\n{}".format(target_T_source_est[0]))
    print("Translation: {}".format(np.squeeze(target_T_source_est[1])))
    print("")
    print("Error")
    print("=====")
    print(
        "Rotation: {}".format(
            get_angular_error(target_T_source[0], target_T_source_est[0])
        )
    )
    print(
        "Translation: {}".format(
            np.linalg.norm(target_T_source[1] - target_T_source_est[1])
        )
    )


def run_example(args, object_graph, solver, sample_position):
    """Run a single pose estimate routine."""
    nodes = object_graph.get_nodes_near_point(sample_position, args.sample_size)
    print("")
    print("Matching")
    print("--------")
    print("Nodes in range: {}".format(len(nodes)))
    descriptors = object_graph.get_descriptors(nodes)
    matches = object_graph.get_matches(descriptors, args.match_threshold)
    print("Matches found: {}".format(len(matches)))
    if len(matches) == 0:
        print("Failed: Not Enough matches")
        return

    source_nodes = [nodes[i] for i in matches]
    positions = object_graph.get_node_positions(source_nodes)

    print("")
    print("Node-Node Correspondences")
    print("-------------------------")
    for index, graph_node_id in matches.items():
        print(
            "match #{}: {} -> {}".format(
                index,
                kimera_dsg.NodeSymbol(nodes[index]),
                kimera_dsg.NodeSymbol(graph_node_id),
            )
        )

    target_T_source = (
        np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]),
        np.zeros((3, 1)),
    )
    perturbed_positions = transform_points(target_T_source, positions)

    target_T_source_est = object_graph.solve_for_pose(
        solver, matches, perturbed_positions
    )
    show_results(target_T_source, target_T_source_est)


def get_args():
    """Set up argument parser and return parsed args."""
    parser = argparse.ArgumentParser(description="benchmarking script for lcd")
    parser.add_argument("graph", help="graph file to use")
    parser.add_argument(
        "--radius",
        "-r",
        type=float,
        help="connection radius for object nodes",
        default=2.0,
    )
    parser.add_argument(
        "--descriptor_size",
        "-n",
        type=int,
        help="number of descriptor rows to use",
        default=60,
    )
    parser.add_argument(
        "--descriptor_length",
        "-l",
        type=int,
        help="size of random walk of descriptor",
        default=3,
    )
    parser.add_argument(
        "--sample_size",
        "-s",
        type=float,
        help="size of sample radius for test pose",
        default=8.0,
    )
    parser.add_argument(
        "--match_threshold",
        "-t",
        type=float,
        help="descriptor matching threshold",
        default=0.05,
    )
    parser.add_argument(
        "--lowes_ratio", type=float, help="lowes ratio for matching", default=0.95
    )
    parser.add_argument(
        "--display_graph", "-d", action="store_true", help="display graph"
    )
    return parser.parse_args()


def main():
    """Construct a solver and run some benchmarks."""
    args = get_args()
    np.set_printoptions(suppress=True, precision=4)

    graph_file = os.path.expanduser(args.graph)
    dsg = kimera_dsg.SceneGraph()
    dsg.load(graph_file)

    params = get_params()
    solver = teaserpp_python.RobustRegistrationSolver(params)

    object_graph = ObjectGraph(
        dsg, args.radius, args.descriptor_size, descriptor_length=args.descriptor_length
    )

    if args.display_graph:
        object_graph.draw_graph()

    sample_position = object_graph.get_centroid()
    print("/" * 80)
    title = "Object-Based LCD Example"
    space = 80 - len(title) - 6
    extra = 0
    if space % 2 == 1:
        space -= 1
        extra = 1
    space /= 2
    print("///{}{}{}///".format(" " * space, title, " " * (space + extra)))
    print("/" * 80)
    print("")
    print("Running example with: {}".format(sample_position))
    run_example(args, object_graph, solver, sample_position)


if __name__ == "__main__":
    main()
