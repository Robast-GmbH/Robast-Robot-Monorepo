import re
import argparse
import os


def generate_error_mapping(output_dir: str) -> None:
    c_header = open("include/error_utils/error_definitions.hpp")

    # Regular expressions to capture error codes and their interfaces
    error_code_pattern = re.compile(r"#define\s+(\w+)\s+(\d+)")
    interface_pattern = re.compile(r"#define\s+(\w+_INTERFACE)\s+(.+)")

    # Dictionaries to hold parsed values
    error_code_map = {}
    interface_map = {}

    # Process each line of the input string
    for line in c_header:
        error_match = error_code_pattern.match(line)
        interface_match = interface_pattern.match(line)

        if error_match:
            error_name, error_value = error_match.groups()
            error_code_map[error_name] = int(error_value)

        if interface_match:
            interface_name, interface_value = interface_match.groups()
            interface_map[interface_name.replace("_INTERFACE", "")] = (
                interface_value.strip()
            )

    # Create the final Python mapping of error codes to interfaces
    error_code_to_interface = {
        value: interface_map[key]
        for key, value in error_code_map.items()
        if key in interface_map
    }

    # Generate the output Python file
    output_py_content = "error_codes_by_interface = {\n"
    for code, interface in error_code_to_interface.items():
        output_py_content += f'    {code}: "{interface}",\n'
    output_py_content += "}\n"

    os.makedirs(output_dir, exist_ok=True)
    # Write to a Python file
    output_path = os.path.join(output_dir, "error_codes.py")
    with open(output_path, "w") as f:
        f.write(output_py_content)

    print("Conversion complete. Check 'error_codes.py'.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate an error mapping file.")
    parser.add_argument(
        "--output_dir",
        default=".",
        help="Directory to save the generated error mapping",
        type=str,
    )
    args = parser.parse_args()

    generate_error_mapping(args.output_dir)
