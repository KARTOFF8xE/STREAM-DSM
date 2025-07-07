import sys

if len(sys.argv) != 4:
    print("Usage: python3 compute_difference.py <file1> <file2> <output_file>")
    sys.exit(1)

file1 = sys.argv[1]
file2 = sys.argv[2]
output_file = sys.argv[3]

try:
    with open(file1, 'r') as f1, open(file2, 'r') as f2:
        lines1 = f1.readlines()
        lines2 = f2.readlines()

    if len(lines1) != len(lines2):
        print("Error: The files have a different number of lines.")
        sys.exit(1)

    differences = []
    for l1, l2 in zip(lines1, lines2):
        try:
            num1 = int(l1.strip())
            num2 = int(l2.strip())
            diff = num2 - num1
            differences.append(f"{diff}\n")
        except ValueError:
            differences.append("NaN\n")
            print(f"Warning: Invalid number found – '{l1.strip()}' or '{l2.strip()}'", file=sys.stderr)

    with open(output_file, 'w') as fout:
        fout.writelines(differences)

    print(f"Differences written to: {output_file}")

except FileNotFoundError as e:
    print(f"File not found: {e}")
    sys.exit(1)

except Exception as e:
    print(f"Unexpected error: {e}")
    sys.exit(1)
