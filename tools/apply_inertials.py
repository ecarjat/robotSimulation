#!/usr/bin/env python3
import argparse
import re
import sys


def parse_args():
    ap = argparse.ArgumentParser(
        description="Apply inertials.xml snippets to a MuJoCo MJCF file."
    )
    ap.add_argument("inertials", help="Path to inertials.xml")
    ap.add_argument("mjcf", help="Path to MJCF file (e.g., robotv2.xml)")
    ap.add_argument("--in-place", action="store_true", help="Modify MJCF in place")
    return ap.parse_args()


def load_inertials(path):
    text = open(path, "r", encoding="utf-8").read()
    text = text.replace("\\n", "\n")
    pattern = re.compile(r"<!--\s*([^\s>]+)\s*-->\s*(<inertial\b[^>]*/>)", re.MULTILINE)
    out = {}
    for name, inertial in pattern.findall(text):
        out[name.strip()] = inertial.strip()
    return out


def replace_inertial(mjcf_text, name, inertial_tag):
    body_pat = re.compile(
        r'(<body\b[^>]*\bname="{n}"[^>]*>)(.*?)(</body>)'.format(n=re.escape(name)),
        re.DOTALL,
    )

    def _sub(match):
        head, body, tail = match.group(1), match.group(2), match.group(3)
        inert_pat = re.compile(r"<inertial\b[^>]*/>")
        if inert_pat.search(body):
            body = inert_pat.sub(inertial_tag, body, count=1)
        else:
            # insert right after the opening body tag
            body = "\n        " + inertial_tag + body
        return head + body + tail

    new_text, count = body_pat.subn(_sub, mjcf_text, count=1)
    return new_text, count


def main():
    args = parse_args()
    inertials = load_inertials(args.inertials)
    if not inertials:
        print("No inertials found in file.", file=sys.stderr)
        sys.exit(1)

    mjcf_text = open(args.mjcf, "r", encoding="utf-8").read()
    changed = 0
    missing = []

    for name, inertial in inertials.items():
        mjcf_text, count = replace_inertial(mjcf_text, name, inertial)
        if count:
            changed += 1
        else:
            missing.append(name)

    if args.in_place:
        with open(args.mjcf, "w", encoding="utf-8") as f:
            f.write(mjcf_text)
    else:
        sys.stdout.write(mjcf_text)

    print(f"Updated {changed} bodies from {args.inertials}.", file=sys.stderr)
    if missing:
        print("Missing bodies:", ", ".join(missing), file=sys.stderr)


if __name__ == "__main__":
    main()
