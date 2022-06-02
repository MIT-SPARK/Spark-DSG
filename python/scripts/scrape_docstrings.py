#!/usr/bin/env python2
"""Make a shim module of the python binding extensions."""
import kimera_dsg_python_bindings as kimera_dsg
import inspect
import sys


def make_function(name, member):
    """Make a function string."""
    output = "    def {}(self):\n".format(name)
    output += '        """{}"""\n'.format(inspect.getdoc(member))
    output += "        pass\n\n"
    return output


def make_property(name, member):
    """Make a property string."""
    doc_str = inspect.getdoc(member)
    if doc_str is None or doc_str == "":
        return "      {}: field {} for class\n\n".format(name, name)
    else:
        doc_str.replace("\n", " ")
        return "      {}: {}\n\n".format(name, doc_str)


def is_private(name):
    """Check if a name is private."""
    if name == "__init__":
        return False

    return name[0] == "_"


def output_class(name, member, output_file):
    """Print a class to stdout."""
    function_output = ""
    property_output = ""
    for submember_name, submember in inspect.getmembers(member):
        if is_private(submember_name):
            continue

        if inspect.ismethod(submember) or submember_name == "__init__":
            function_output += make_function(submember_name, submember)
        elif isinstance(submember, member):
            continue  # enum member already documented
        else:
            property_output += make_property(submember_name, submember)

    output = "class {}:\n".format(name)
    if inspect.getdoc(member) is None:
        output += '    """\n'
    else:
        output += '    """\n{}\n\n'.format(inspect.getdoc(member))

    if property_output != "":
        output += "    Members:\n"
        output += property_output + "\n"

    output += '    """\n'

    if function_output != "":
        output += function_output
    else:
        output += "    pass\n"

    output_file.write(output)


def export_member(name, member, output_file):
    """Export a member to the module file."""
    output_class(name, member, output_file)


def main():
    """Do everything."""
    if len(sys.argv) < 2:
        print("invalid number of arguments.")

    with open(sys.argv[1], "w") as output_file:
        for name, member in inspect.getmembers(kimera_dsg):
            if inspect.isfunction(member) or inspect.isclass(member):
                export_member(name, member, output_file)


if __name__ == "__main__":
    main()
