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
"""Visualization for a DSG."""

import logging

import numpy as np

# DSG plot style
LAYER_TYPE_OFFSET = {
    5: 30,  # buildings
    4: 25,  # rooms
    3: 10,  # places
    2: 0,  # objects
}

LAYER_TYPE_TO_COLOR = {
    5: "#636EFA",  # buildings
    4: "#EF553B",  # rooms
    3: "#AB63FA",  # places
    2: "#00CC96",  # objects
}


def z_offset(layer_idx: int) -> np.ndarray:
    return np.array([0, 0, LAYER_TYPE_OFFSET[layer_idx]])


def _draw_layer_nodes(
    fig,
    layer,
    marker_size=12,
    annotation_size=18,
    include_text=False,
    text_func=None,
    color_func=None,
):
    import plotly.graph_objects as go

    pos, colors, text = [], [], []

    for node in layer.nodes:
        pos.append(np.array(node.attributes.position) + z_offset(layer.id))
        if color_func is None:
            colors.append(LAYER_TYPE_TO_COLOR[layer.id])
        else:
            colors.append(color_func(node))
        if text_func is None:
            text.append(str(node.id))
        else:
            text.append(text_func(node))

    if len(pos) == 0:
        return

    pos = np.array(pos)
    fig.add_trace(
        go.Scatter3d(
            x=pos[:, 0],
            y=pos[:, 1],
            z=pos[:, 2],
            text=text if include_text else "",
            mode="markers",
            marker=dict(color=colors, size=marker_size, opacity=0.8),
            # textfont=dict(size=annotation_size),
            # hoverinfo="none",
        )
    )


def plot_scene_graph(G, title=None, figure_path=None, layer_settings=None):
    """Construct and returns a 3D scattter plot of a DSG."""
    try:
        import plotly.graph_objects as go
    except ImportError:
        logging.warning(
            "plotly not found. Install spark_dsg with the viz extra to for plotting!"
        )
        return None

    fig = go.Figure()
    for layer in G.layers:
        has_settings = layer_settings is not None and layer.id in layer_settings
        settings = {} if not has_settings else layer_settings[layer.id]
        if layer.id in LAYER_TYPE_OFFSET.keys():
            _draw_layer_nodes(fig, layer, **settings)
        else:
            print(
                f"Skipping layer (id={layer.id}), "
                f"containing {len([n for n in layer.nodes])} nodes."
            )

    # edges
    x_lines, y_lines, z_lines = [], [], []
    x_lines_dark, y_lines_dark, z_lines_dark = [], [], []

    for edge in G.edges:
        source = G.get_node(edge.source)
        target = G.get_node(edge.target)

        if (
            source.layer.layer not in LAYER_TYPE_OFFSET.keys()
            or target.layer.layer not in LAYER_TYPE_OFFSET.keys()
        ):
            continue
        source_pos = np.array(source.attributes.position) + z_offset(source.layer.layer)
        target_pos = np.array(target.attributes.position) + z_offset(target.layer.layer)

        # intralayer edges
        if source.layer == target.layer:
            x_lines_dark += [source_pos[0], target_pos[0], None]
            y_lines_dark += [source_pos[1], target_pos[1], None]
            z_lines_dark += [source_pos[2], target_pos[2], None]

        # interlayer edges
        else:
            x_lines += [source_pos[0], target_pos[0], None]
            y_lines += [source_pos[1], target_pos[1], None]
            z_lines += [source_pos[2], target_pos[2], None]

    # add interlayer edges to plot
    fig.add_trace(
        go.Scatter3d(
            x=x_lines_dark,
            y=y_lines_dark,
            z=z_lines_dark,
            mode="lines",
            line=dict(color="rgb(0,0,0)", width=1),
            hoverinfo="none",
        )
    )

    # add intralayer edges to plot
    fig.add_trace(
        go.Scatter3d(
            x=x_lines,
            y=y_lines,
            z=z_lines,
            mode="lines",
            line=dict(color="rgb(125,125,125)", width=1),
            hoverinfo="none",
        )
    )

    # layout and background
    axis = dict(
        showbackground=False,
        showline=False,
        zeroline=False,
        showgrid=False,
        showticklabels=False,
        title="",
    )

    fig.layout = go.Layout(
        title=title,
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(0,0,0,0)",
        showlegend=False,
        scene=dict(
            xaxis=dict(axis),
            yaxis=dict(axis),
            zaxis=dict(axis),
        ),
        margin=dict(t=100),
        hovermode="closest",
    )

    if figure_path is not None:
        fig.write_html(figure_path)
    return fig
