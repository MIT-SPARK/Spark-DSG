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
"""Conversion from spark_dsg to networkx."""
import importlib
import logging


def _get_networkx():
    networkx = None
    try:
        networkx = importlib.import_module("networkx")
    except ImportError:
        logging.warning("networkx not found. conversion disabled")

    return networkx


def _convert_attr(attrs):
    valid_fields = [x for x in dir(attrs) if x[0] != "_"]
    return {x: getattr(attrs, x) for x in valid_fields}


def _fill_from_layer(G_out, layer):
    for node in layer.nodes:
        G_out.add_node(node.id.value, **_convert_attr(node.attributes))

    for edge in layer.edges:
        G_out.add_edge(edge.source, edge.target, **_convert_attr(edge.info))


def graph_to_networkx(G_in, include_dynamic=True):
    """Convert the DSG to a networkx representation."""
    nx = _get_networkx()
    if nx is None:
        return None

    G_out = nx.Graph()
    for layer in G_in.layers:
        _fill_from_layer(G_out, layer)

    for edge in G_in.interlayer_edges:
        G_out.add_edge(edge.source, edge.target, **_convert_attr(edge.info))

    if not include_dynamic:
        return G_out

    for layer in G_in.dynamic_layers:
        _fill_from_layer(G_out, layer)

    for edge in G_in.dynamic_interlayer_edges:
        G_out.add_edge(edge.source, edge.target, **_convert_attr(edge.info))

    return G_out


def layer_to_networkx(G_in):
    """Convert the DSG to a networkx representation."""
    nx = _get_networkx()
    if nx is None:
        return None

    G_out = nx.Graph()
    _fill_from_layer(G_out, G_in)
    return G_out
