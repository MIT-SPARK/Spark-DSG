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
import numpy as np
import logging

# DSG plot style
NODE_TYPE_OFFSET = {"B": 30, "R": 25, "p": 10, "O": 0}

NODE_TYPE_TO_COLOR = {"B": "#636EFA", "R": "#EF553B", "p": "#AB63FA", "O": "#00CC96"}


def z_offset(node) -> np.ndarray:
    """Take a node and returns an offset in the z direction according to node type."""
    offset = node.attributes.position.copy()
    offset[2] += NODE_TYPE_OFFSET[node.id.category]
    return offset


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
        pos.append(np.squeeze(z_offset(node)))
        if color_func is None:
            colors.append(NODE_TYPE_TO_COLOR[node.id.category])
        else:
            colors.append(color_func(node))

        if text_func is None:
            text.append(str(node.id))
        else:
            text.append(text_func(node))

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
        print(settings)
        _draw_layer_nodes(fig, layer, **settings)

    # edges
    x_lines, y_lines, z_lines = [], [], []
    x_lines_dark, y_lines_dark, z_lines_dark = [], [], []

    for edge in G.edges:
        source = G.get_node(edge.source)
        target = G.get_node(edge.target)

        start_offset = z_offset(source)
        end_offset = z_offset(target)

        # intralayer edges
        if source.layer == target.layer:
            x_lines_dark += [start_offset[0], end_offset[0], None]
            y_lines_dark += [start_offset[1], end_offset[1], None]
            z_lines_dark += [start_offset[2], end_offset[2], None]

        # interlayer edges
        else:
            x_lines += [start_offset[0], end_offset[0], None]
            y_lines += [start_offset[1], end_offset[1], None]
            z_lines += [start_offset[2], end_offset[2], None]

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
