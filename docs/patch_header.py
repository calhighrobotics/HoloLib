#!/usr/bin/env python3
"""Patch the Doxygen-generated header.html to enable doxygen-awesome features.

Doxygen generates a default `header.html` from the active version's template
(via `doxygen -w`). We inject the doxygen-awesome JavaScript extensions just
before </head> so the version of the template always matches the installed
Doxygen, while still getting the dark-mode toggle, fragment copy buttons,
paragraph links, interactive table of contents and tabbed sections.

Run from the `docs/` directory after `doxygen -w html header.html ...`.
"""

from pathlib import Path

HEADER = Path("header.html")

INJECT = """\
<!-- doxygen-awesome-css extensions (injected by patch_header.py) -->
<script type="text/javascript" src="$relpath^doxygen-awesome-darkmode-toggle.js"></script>
<script type="text/javascript" src="$relpath^doxygen-awesome-fragment-copy-button.js"></script>
<script type="text/javascript" src="$relpath^doxygen-awesome-paragraph-link.js"></script>
<script type="text/javascript" src="$relpath^doxygen-awesome-interactive-toc.js"></script>
<script type="text/javascript" src="$relpath^doxygen-awesome-tabs.js"></script>
<script type="text/javascript">
  DoxygenAwesomeDarkModeToggle.init()
  DoxygenAwesomeFragmentCopyButton.init()
  DoxygenAwesomeParagraphLink.init()
  DoxygenAwesomeInteractiveToc.init()
  DoxygenAwesomeTabs.init()
</script>
<!-- HoloLib title hero + scroll-reveal (self-initializing) -->
<script type="text/javascript" src="$relpath^holo-enhance.js"></script>
"""


def main() -> None:
    html = HEADER.read_text(encoding="utf-8")

    if "doxygen-awesome-darkmode-toggle.js" in html:
        print("header.html already patched; skipping.")
        return

    if "</head>" not in html:
        raise SystemExit("Could not find </head> in header.html")

    html = html.replace("</head>", INJECT + "</head>", 1)
    HEADER.write_text(html, encoding="utf-8")
    print("Patched header.html with doxygen-awesome extensions.")


if __name__ == "__main__":
    main()
