#!/usr/bin/env python3
"""Convert G_PhrozenFluiddRespondInfo debug calls to kaos_log("DEBUG", ...).

Rules:
- KEEP calls where the string argument starts with "+" (protocol messages)
- KEEP calls where the string starts with "V-H" (version string)
- KEEP the assignment line in base.py (G_PhrozenFluiddRespondInfo = ...)
- Convert everything else to self.kaos_log("DEBUG", ..., "SERIAL")

Usage:
    python3 tools/convert_to_kaos_log.py phrozen_dev/cmds.py phrozen_dev/base.py
"""

import re
import sys

# Matches: self.G_PhrozenFluiddRespondInfo(
CALL_PATTERN = re.compile(r"^(\s*)self\.G_PhrozenFluiddRespondInfo\(")

# Protocol: first string literal starts with + or V-H
PROTOCOL_RE = re.compile(r"""^\s*["']\+|^\s*["']V-H""")


def is_protocol_arg(arg_text):
    """Check if the argument text starts with a protocol prefix."""
    stripped = arg_text.strip()
    # Direct string: "+AMSERROR:2" or "V-H%s..."
    if re.match(r'^["\']\+', stripped):
        return True
    if re.match(r'^["\']V-H', stripped):
        return True
    return False


def find_matching_paren(lines, start_line, start_col):
    """Find the line and column of the matching closing paren."""
    depth = 1
    line_idx = start_line
    col = start_col

    while line_idx < len(lines):
        line = lines[line_idx]
        while col < len(line):
            ch = line[col]
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
                if depth == 0:
                    return line_idx, col
            elif ch in ('"', "'"):
                # Skip string literals
                quote = ch
                col += 1
                # Check for triple-quote
                if col + 1 < len(line) and line[col : col + 2] == quote * 2:
                    # Triple-quoted string
                    col += 2
                    end_triple = quote * 3
                    while line_idx < len(lines):
                        pos = lines[line_idx].find(end_triple, col)
                        if pos != -1:
                            col = pos + 3
                            break
                        line_idx += 1
                        col = 0
                    continue
                else:
                    # Single-quoted string - find end
                    while col < len(line):
                        if line[col] == "\\":
                            col += 2
                            continue
                        if line[col] == quote:
                            col += 1
                            break
                        col += 1
                    continue
            elif ch == "#":
                # Rest of line is comment
                break
            col += 1
        line_idx += 1
        col = 0

    return None, None


def transform_file(filepath):
    """Transform a single file."""
    with open(filepath, "r") as f:
        lines = f.readlines()

    output = []
    i = 0
    converted = 0
    skipped_protocol = 0
    skipped_assign = 0

    while i < len(lines):
        line = lines[i]

        # Skip assignment lines (base.py: G_PhrozenFluiddRespondInfo = ...)
        if "G_PhrozenFluiddRespondInfo =" in line or "G_PhrozenFluiddRespondInfo=" in line:
            output.append(line)
            skipped_assign += 1
            i += 1
            continue

        # Check for the call pattern
        match = CALL_PATTERN.search(line)
        if not match:
            output.append(line)
            i += 1
            continue

        indent = match.group(1)
        # Find the opening paren position
        open_paren_col = match.end() - 1  # position of '('

        # Find matching close paren
        end_line, end_col = find_matching_paren(lines, i, open_paren_col + 1)

        if end_line is None:
            # Couldn't find matching paren - leave unchanged
            output.append(line)
            i += 1
            continue

        # Extract the argument text (between the parens)
        if end_line == i:
            # Single-line call
            arg_text = line[open_paren_col + 1 : end_col]
        else:
            # Multi-line: first partial line + middle lines + last partial line
            arg_parts = [line[open_paren_col + 1 :]]
            for mid in range(i + 1, end_line):
                arg_parts.append(lines[mid])
            arg_parts.append(lines[end_line][:end_col])
            arg_text = "".join(arg_parts)

        # Check if this is a protocol message
        if is_protocol_arg(arg_text):
            # Keep as-is
            for line_idx in range(i, end_line + 1):
                output.append(lines[line_idx])
            skipped_protocol += 1
            i = end_line + 1
            continue

        # Convert to kaos_log
        # Reconstruct: self.kaos_log("DEBUG", <arg>, "SERIAL")
        if end_line == i:
            # Single-line call
            after = line[end_col + 1 :]  # after the closing paren (includes newline)
            arg = line[open_paren_col + 1 : end_col].strip()
            new_line = '%sself.kaos_log("DEBUG", %s, "SERIAL")%s' % (
                indent,
                arg,
                after,
            )
            output.append(new_line)
        else:
            # Multi-line call
            # Determine the "inner indent" from the first argument line
            inner_indent = indent + "    "
            if i + 1 < len(lines):
                first_content = lines[i + 1]
                stripped = first_content.lstrip()
                if stripped:
                    inner_indent = first_content[: len(first_content) - len(stripped)]

            # First line: replace function name
            output.append('%sself.kaos_log("DEBUG",\n' % indent)

            # Check if end_line is just whitespace + ")"
            last_line = lines[end_line]
            last_before_paren = last_line[:end_col]
            last_after = last_line[end_col + 1 :]

            if last_before_paren.strip() == "":
                # Closing paren is on its own line
                # Emit middle lines, but add comma to the last content line
                middle_lines = list(range(i + 1, end_line))
                for idx, mid in enumerate(middle_lines):
                    if idx == len(middle_lines) - 1:
                        # Last content line - add trailing comma
                        content = lines[mid].rstrip("\n")
                        output.append("%s,\n" % content)
                    else:
                        output.append(lines[mid])
                # Replace the closing-paren line with "SERIAL")
                output.append('%s"SERIAL")%s' % (inner_indent, last_after))
            else:
                # Closing paren is on the same line as content
                # Emit middle lines as-is
                for mid in range(i + 1, end_line):
                    output.append(lines[mid])
                # Append , "SERIAL" before the closing paren
                content_part = last_before_paren.rstrip()
                output.append('%s, "SERIAL")%s' % (content_part, last_after))

        converted += 1
        i = end_line + 1

    with open(filepath, "w") as f:
        f.writelines(output)

    print(
        "  %s: converted=%d, skipped_protocol=%d, skipped_assign=%d"
        % (filepath, converted, skipped_protocol, skipped_assign)
    )
    return converted


def main():
    if len(sys.argv) < 2:
        print("Usage: %s <file1.py> [file2.py ...]" % sys.argv[0])
        sys.exit(1)

    total = 0
    for filepath in sys.argv[1:]:
        print("Processing: %s" % filepath)
        total += transform_file(filepath)

    print("\nTotal converted: %d" % total)


if __name__ == "__main__":
    main()
