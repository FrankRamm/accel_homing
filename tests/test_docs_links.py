import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SKIP_DIRS = {".git", "validation", "__pycache__"}
LINK_RE = re.compile(r"\]\(([^)\s]+)\)")
HEADING_RE = re.compile(r"^#{1,6}\s+(.*?)\s*#*\s*$")
FENCE_RE = re.compile(r"^\s*(```|~~~)")


def github_slug(heading):
    s = heading.strip().lower()
    s = re.sub(r"[^\w\- ]", "", s)
    return s.replace(" ", "-")


def markdown_files():
    result = []
    for path in sorted(ROOT.rglob("*.md")):
        rel = path.relative_to(ROOT)
        if SKIP_DIRS & set(rel.parts):
            continue
        if rel.parts[:2] == ("docs", "superpowers"):
            continue
        if ".superpowers" in rel.parts:
            continue
        result.append(path)
    return result


def unfenced_lines(text):
    in_fence = False
    for line in text.splitlines():
        if FENCE_RE.match(line):
            in_fence = not in_fence
            continue
        if not in_fence:
            yield line


def heading_slugs(path):
    slugs = set()
    seen = {}
    for line in unfenced_lines(path.read_text(encoding="utf-8")):
        m = HEADING_RE.match(line)
        if not m:
            continue
        slug = github_slug(m.group(1))
        count = seen.get(slug, 0)
        seen[slug] = count + 1
        slugs.add(slug if count == 0 else "%s-%d" % (slug, count))
    return slugs


class DocsLinksTest(unittest.TestCase):
    def test_relative_markdown_links_resolve(self):
        failures = []
        for md in markdown_files():
            rel = md.relative_to(ROOT).as_posix()
            for line in unfenced_lines(md.read_text(encoding="utf-8")):
                for target in LINK_RE.findall(line):
                    if target.startswith(("http:", "https:", "mailto:")):
                        continue
                    path_part, _, anchor = target.partition("#")
                    dest = md if not path_part else (md.parent / path_part).resolve()
                    if not dest.exists():
                        failures.append("%s: %s" % (rel, target))
                        continue
                    if anchor and dest.suffix == ".md":
                        if anchor.lower() not in heading_slugs(dest):
                            failures.append("%s: %s" % (rel, target))
        self.assertEqual([], failures)

    def test_example_config_doc_paths_resolve(self):
        failures = []
        for cfg in sorted((ROOT / "examples").glob("*.cfg")):
            text = cfg.read_text(encoding="utf-8")
            for m in re.finditer(r"(?:\.\./)?(docs/[\w./-]+\.md)", text):
                if not (ROOT / m.group(1)).exists():
                    failures.append("%s: %s" % (cfg.name, m.group(1)))
        self.assertEqual([], failures)

    def test_no_legacy_uppercase_doc_paths(self):
        pattern = re.compile(r"docs/[A-Z_]+\.md|\]\((?:\.\./)?TESTING\.md")
        failures = []
        files = markdown_files() + sorted((ROOT / "examples").glob("*.cfg"))
        for path in files:
            text = path.read_text(encoding="utf-8")
            for m in pattern.finditer(text):
                failures.append("%s: %s" % (path.relative_to(ROOT).as_posix(), m.group(0)))
        self.assertEqual([], failures)


if __name__ == "__main__":
    unittest.main()
