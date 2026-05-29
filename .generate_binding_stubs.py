#!/usr/bin/env python3
import pathlib
import re
import subprocess

QUIRKS = [
    (r"spark_dsg\._dsg_bindings\.", "", 0),
    (r"__eq__\(self, arg0: .+\)", "__eq__(self, arg0: typing.Any)", 0),
    (r"__ne__\(self, arg0: .+\)", "__ne__(self, arg0: typing.Any)", 0),
    (r"import datetime\n", "import datetime\nimport os\n", 0),
    (r": PartitionId", ": PartitionId | int", 0),
    (r": NodeSymbol", ": NodeSymbol | int", 0),
    (r"(@attributes\.setter\n.*?)\) -> NodeAttributes", r"\1, NodeAttributes)", 0),
    (r"(\n    def str\(self,.*?:\n.*?\.\.\.)(.*?)(\nclass )", r"\2\1\3", re.DOTALL),
    (r"(def find_node\(.*-> )SceneGraphNode:", r"\1SceneGraphNode | None:", 0),
    (r"(def find_edge\(.*-> )SceneGraphEdge:", r"\1SceneGraphEdge | None:", 0),
]


def main():
    filepath = pathlib.Path(__file__).absolute().parent
    cmd = ["pybind11-stubgen", "spark_dsg._dsg_bindings", "-o", "python"]
    subprocess.run(cmd, cwd=str(filepath))

    type_path = filepath / "python" / "spark_dsg" / "_dsg_bindings.pyi"
    with type_path.open("r") as fin:
        contents = fin.read()

    for pattern, replacement, flags in QUIRKS:
        matcher = re.compile(pattern, flags)
        contents = matcher.sub(replacement, contents)

    with type_path.open("w") as fin:
        fin.write(contents)

    cmd = [
        "ruff",
        "check",
        "--silent",
        "--select",
        "I",
        "--fix",
        "python/spark_dsg/_dsg_bindings.pyi",
    ]
    subprocess.run(cmd, cwd=str(filepath))
    cmd = ["ruff", "format", "--silent", "python/spark_dsg/_dsg_bindings.pyi"]
    subprocess.run(cmd, cwd=str(filepath))


if __name__ == "__main__":
    main()
