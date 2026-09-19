#!/usr/bin/env python3
"""Open a pull request on the website repo publishing a firmware release.

Run from CI after the release is published (see .github/workflows/build.yml).
It drops the new .uf2 into blhall195/DiscoX-Cave-Survey-Device, rewrites the
four version strings on firmware.html, adds a changelog entry built from the
GitHub release notes, and opens a PR. Brendan reviews the diff and merges;
merging to that repo's main is a deploy to discox.co.uk.

This is a script rather than inline YAML for one reason: it can be dry-run
against a real checkout on a laptop, so the string rewriting is verifiable
without pushing a tag.

    # prove it reproduces what is already live (expect an empty diff, 4/4 found)
    python3 tools/publish_to_website.py --tag v2.0.2 --dry-run \
        --website-dir ../DiscoX-Cave-Survey-Device

    # see what a real release would change
    python3 tools/publish_to_website.py --tag v2.0.3 --dry-run \
        --website-dir ../DiscoX-Cave-Survey-Device \
        --uf2 build/mrzappy-v2.0.3.uf2 --notes notes.md
"""

import argparse
import difflib
import hashlib
import html
import os
import re
import shutil
import subprocess
import sys
import tempfile
from datetime import date, datetime, timezone
from pathlib import Path

FIRMWARE_REPO = "blhall195/DiscoX-V2"
WEBSITE_REPO = "blhall195/DiscoX-Cave-Survey-Device"
PAGE = "firmware.html"

# Keep the newest N vendored .uf2 files at the website root. Each is ~835 KB, so
# without this the repo gains a binary per release for ever. Note that deleting
# from the working tree does not shrink git history -- this only stops it
# getting worse.
DEFAULT_KEEP = 2

# firmware.uf2 is the V1 device's firmware, linked from the data-ver="v1"
# blocks. It is not part of the V2 release train and must never be pruned.
V2_UF2 = re.compile(r"^mrzappy-v.+\.uf2$")


def run(cmd, cwd=None, capture=False, check=True, env=None):
    """Run a command. Never logs `cmd` on failure -- it can carry a token."""
    return subprocess.run(
        cmd, cwd=cwd, check=check, env=env, text=True,
        stdout=subprocess.PIPE if capture else None,
    )


def fail(msg):
    print(f"error: {msg}", file=sys.stderr)
    sys.exit(1)


def note(msg):
    print(f"  {msg}")


# --------------------------------------------------------------------------
# version helpers


def version_key(name):
    """Sort key for mrzappy-vX.Y.Z[-suffix].uf2. Unparseable sorts last."""
    m = re.match(r"^mrzappy-v(\d+)\.(\d+)\.(\d+)(.*)\.uf2$", name)
    if not m:
        return (1, (0, 0, 0), name)
    major, minor, patch, suffix = m.groups()
    # A bare release outranks its own pre-releases: "" sorts after "-rc1".
    return (0, (int(major), int(minor), int(patch), suffix == "", suffix), name)


def is_prerelease(tag):
    return not re.match(r"^v?\d+\.\d+\.\d+$", tag)


def short_sha(path):
    """First four and last four hex characters, the form firmware.html uses."""
    digest = hashlib.sha256(Path(path).read_bytes()).hexdigest()
    return f"{digest[:4]}…{digest[-4:]}", digest


# --------------------------------------------------------------------------
# release notes -> <li> items


AUTOGEN_BULLET = re.compile(r"by @[\w-]+ in https://github\.com/")


def inline_md(text):
    """Escape, then re-apply the handful of inline forms the notes actually use."""
    out = html.escape(text, quote=False)
    out = re.sub(r"`([^`]+)`", r"<code>\1</code>", out)
    out = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", out)
    out = re.sub(r"\[([^\]]+)\]\((https?://[^)]+)\)", r'<a href="\2">\1</a>', out)
    return out


def notes_to_items(body):
    """Top-level markdown bullets -> list of HTML <li> inner strings.

    Returns [] when the body is missing, has no bullets, or is GitHub's
    auto-generated commit list. Commit titles are not customer-facing copy and
    must never reach the site, so an unusable body produces no changelog entry
    at all rather than a bad one.
    """
    if not body or not body.strip():
        return []

    items, current = [], None
    for line in body.splitlines():
        if re.match(r"^\s*\*\*Full Changelog\*\*", line):
            break
        if line.startswith("#"):
            current = None
            continue
        m = re.match(r"^[-*]\s+(.*)$", line)
        if m:
            current = [m.group(1).strip()]
            items.append(current)
        elif current is not None and line.strip() and line.startswith((" ", "\t")):
            current.append(line.strip())  # wrapped continuation of a bullet
        elif not line.strip():
            current = None

    joined = [" ".join(parts).strip() for parts in items]
    joined = [t for t in joined if t]
    if not joined:
        return []
    if all(AUTOGEN_BULLET.search(t) for t in joined):
        return []  # GitHub's generated "* thing by @who in <url>" list
    return [inline_md(t) for t in joined]


def fetch_notes(tag):
    if not shutil.which("gh"):
        note("gh not on PATH -- cannot read the release notes")
        return ""
    try:
        r = run(["gh", "release", "view", tag, "--repo", FIRMWARE_REPO,
                 "--json", "body", "-q", ".body"], capture=True, check=True)
        return r.stdout
    except subprocess.CalledProcessError:
        note(f"no GitHub release found for {tag}")
        return ""


# --------------------------------------------------------------------------
# the four rewrites on firmware.html


def sub_once(pattern, repl, text, what):
    """Substitute exactly one match, or fail. A silent zero-match rewrite here
    would ship a page still advertising the previous firmware."""
    new, n = pattern.subn(repl, text, count=0)
    if n != 1:
        fail(f"{what}: expected exactly 1 match in {PAGE}, found {n}. "
             "The page markup has changed -- update this script.")
    return new


def rewrite_page(text, tag, dt, uf2_name, download_name, sha_short):
    """Returns (new_text, changelog_inserted). Raises via fail() on a miss."""
    text = sub_once(
        re.compile(r'href="mrzappy-[^"]*\.uf2"(\s+)download="DiscoX2-[^"]*\.uf2"'),
        lambda m: f'href="{uf2_name}"{m.group(1)}download="{download_name}"',
        text, "download link")

    text = sub_once(
        re.compile(r"Download DiscoX2-\S*\.uf2"),
        f"Download {download_name}", text, "download link text")

    # Scoped to the data-ver="v2" paragraph: there is an identically shaped line
    # for the V1 device a few lines further down, and rewriting that one would
    # point original-device owners at new-device firmware.
    text = sub_once(
        re.compile(r'(<p data-ver="v2"[^>]*>\s*)'
                   r"v[\d][^\s·]*\s+·\s+released\s+\d{4}-\d{2}-\d{2}"
                   r"\s+·\s+sha256\s+<code>[^<]*</code>"),
        lambda m: (f"{m.group(1)}{tag} · released {dt} "
                   f"· sha256 <code>{sha_short}</code>"),
        text, "version/sha line")

    text = sub_once(
        re.compile(r'(<div class="dropzone__file" data-ver="v2">)'
                   r'DiscoX2-[^<]*\.uf2(</div>)'),
        lambda m: f"{m.group(1)}{download_name}{m.group(2)}",
        text, "dropzone filename")

    return text


def insert_changelog(text, tag, dt, items):
    """Insert a new changelog block above the newest data-ver="v2" heading.

    Idempotent: a heading for this version already present means the entry was
    written on an earlier run, so nothing is inserted.
    """
    if re.search(rf'<h3 data-ver="v2">{re.escape(tag)}\s', text):
        note(f"changelog already has an entry for {tag} -- left alone")
        return text, False

    m = re.search(r'\n([ \t]*)<h3 data-ver="v2">', text)
    if not m:
        fail('changelog: no <h3 data-ver="v2"> heading found in ' + PAGE)

    pad = m.group(1)
    lis = "\n".join(f"{pad}  <li>{i}</li>" for i in items)
    # Trailing blank line: entries in the changelog are separated by one, and a
    # diff that silently closes that gap up reads as an unrelated change.
    block = (f'\n{pad}<h3 data-ver="v2">{tag} — {dt}</h3>\n'
             f'{pad}<ul data-ver="v2">\n{lis}\n{pad}</ul>\n\n')
    return text[:m.start()] + block + text[m.start() + 1:], True


# --------------------------------------------------------------------------


def prune(website, keep, tag):
    """Delete vendored .uf2 files beyond the newest `keep`."""
    if is_prerelease(tag):
        note(f"{tag} is a pre-release -- skipping the .uf2 prune")
        return []
    files = sorted((p for p in website.iterdir() if V2_UF2.match(p.name)),
                   key=lambda p: version_key(p.name), reverse=True)
    doomed = files[keep:]
    for p in doomed:
        note(f"pruning {p.name}")
        p.unlink()
    return [p.name for p in doomed]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tag", required=True, help="release tag, e.g. v2.0.3")
    ap.add_argument("--uf2", help="path to the built .uf2 "
                                  "(default build/mrzappy-<tag>.uf2)")
    ap.add_argument("--notes", help="file holding the release notes, or - for "
                                    "stdin (default: read from the GitHub release)")
    ap.add_argument("--date", help="release date YYYY-MM-DD (default: today, UTC)")
    ap.add_argument("--website-dir", help="existing checkout to work in "
                                          "(default: clone the website repo)")
    ap.add_argument("--website-repo", default=WEBSITE_REPO)
    ap.add_argument("--keep", type=int, default=DEFAULT_KEEP)
    ap.add_argument("--dry-run", action="store_true",
                    help="print the diff and exit; writes nothing, pushes nothing")
    args = ap.parse_args()

    tag = args.tag if args.tag.startswith("v") else "v" + args.tag
    dt = args.date or datetime.now(timezone.utc).date().isoformat()
    try:
        date.fromisoformat(dt)
    except ValueError:
        fail(f"--date must be YYYY-MM-DD, got {dt!r}")

    uf2_name = f"mrzappy-{tag}.uf2"
    download_name = f"DiscoX2-{tag}.uf2"
    repo_root = Path(__file__).resolve().parent.parent

    # --- where we are working -------------------------------------------
    tmp = None
    if args.website_dir:
        website = Path(args.website_dir).resolve()
        if not (website / PAGE).exists():
            fail(f"{website} does not look like the website repo (no {PAGE})")
    else:
        if args.dry_run:
            fail("--dry-run needs --website-dir; it will not clone")
        token = os.environ.get("GH_TOKEN") or os.environ.get("GITHUB_TOKEN")
        if not token:
            fail("GH_TOKEN is not set -- needed to clone and push the website repo")
        tmp = tempfile.mkdtemp(prefix="discox-website-")
        website = Path(tmp) / "website"
        print(f"==> cloning {args.website_repo}")
        # The token is built here and never printed; run() does not echo argv.
        run(["git", "clone", "--depth", "1",
             f"https://x-access-token:{token}@github.com/{args.website_repo}.git",
             str(website)])

    # --- the binary -------------------------------------------------------
    if args.uf2:
        uf2 = Path(args.uf2).resolve()
    else:
        built = repo_root / "build" / uf2_name
        already = website / uf2_name
        if built.exists():
            uf2 = built
        elif args.dry_run and already.exists():
            # Dry-running a version that is already published: re-derive from the
            # copy on the site, so the run proves the script reproduces it.
            note(f"using the already-published {uf2_name} from the website repo")
            uf2 = already
        else:
            fail(f"no {built} -- run ./build.sh first, or pass --uf2")
    if not uf2.exists():
        fail(f"{uf2} does not exist")

    sha_short, digest = short_sha(uf2)
    print(f"==> {tag}  {dt}  {uf2_name}")
    note(f"sha256 {digest}")
    note(f"page shows {sha_short}")

    # --- release notes ----------------------------------------------------
    if args.notes == "-":
        body = sys.stdin.read()
    elif args.notes:
        body = Path(args.notes).read_text()
    else:
        body = fetch_notes(tag)
    items = notes_to_items(body)
    if items:
        note(f"{len(items)} changelog item(s) from the release notes")
    else:
        note("no usable release notes -- the changelog will NOT be updated")

    # --- rewrite ----------------------------------------------------------
    page = website / PAGE
    original = page.read_text()
    text = rewrite_page(original, tag, dt, uf2_name, download_name, sha_short)
    note("4/4 version strings located and rewritten")

    inserted = False
    if items:
        text, inserted = insert_changelog(text, tag, dt, items)

    if args.dry_run:
        diff = difflib.unified_diff(original.splitlines(keepends=True),
                                    text.splitlines(keepends=True),
                                    fromfile=f"a/{PAGE}", tofile=f"b/{PAGE}")
        out = "".join(diff)
        print()
        print(out if out else
              f"(no change to {PAGE} -- the page already describes {tag})")
        print("\n--- dry run: nothing written, nothing pushed ---")
        return

    page.write_text(text)
    if uf2.resolve() != (website / uf2_name).resolve():
        shutil.copy2(uf2, website / uf2_name)
    pruned = prune(website, args.keep, tag)

    # --- branch, commit, PR ----------------------------------------------
    branch = f"firmware-{tag}"
    run(["git", "-C", str(website), "config", "user.name", "github-actions[bot]"])
    run(["git", "-C", str(website), "config", "user.email",
         "41898282+github-actions[bot]@users.noreply.github.com"])
    run(["git", "-C", str(website), "checkout", "-B", branch])
    run(["git", "-C", str(website), "add", "-A"])

    status = run(["git", "-C", str(website), "status", "--porcelain"],
                 capture=True).stdout.strip()
    if not status:
        print("==> website already up to date -- nothing to do")
        return

    run(["git", "-C", str(website), "commit", "-m",
         f"Publish {tag} firmware"])
    run(["git", "-C", str(website), "push", "--force", "origin", branch])

    warn = "" if inserted else (
        "\n> **The changelog was not updated.** The release notes for this tag "
        "are empty or are GitHub's auto-generated commit list, and commit "
        "titles are not customer-facing copy. Write proper release notes on "
        "the GitHub release and re-run the workflow, or add the "
        "`<h3 data-ver=\"v2\">` entry in this PR by hand.\n")

    body_md = (
        f"Published by `tools/publish_to_website.py` from "
        f"[{FIRMWARE_REPO}@{tag}](https://github.com/{FIRMWARE_REPO}/releases/tag/{tag}).\n"
        f"{warn}\n"
        f"- `{uf2_name}` added (sha256 `{digest}`)\n"
        f"- download link, link text, version/sha line and flashing-animation "
        f"filename updated on `{PAGE}`\n"
        f"- changelog entry: {'added' if inserted else 'NOT added'}\n"
        + (f"- pruned: {', '.join(pruned)}\n" if pruned else "")
        + "\nMerging this deploys to discox.co.uk. Check the download link "
          "resolves and the sha matches before you do.\n"
    )

    try:
        r = run(["gh", "pr", "create", "--repo", args.website_repo,
                 "--base", "main", "--head", branch,
                 "--title", f"Publish {tag} firmware", "--body", body_md],
                capture=True)
        print(f"==> {r.stdout.strip()}")
    except subprocess.CalledProcessError:
        r = run(["gh", "pr", "list", "--repo", args.website_repo,
                 "--head", branch, "--json", "url", "-q", ".[0].url"],
                capture=True, check=False)
        url = r.stdout.strip() if r.stdout else ""
        print(f"==> updated existing PR {url}" if url else
              "==> branch pushed, but no PR could be created -- open one by hand")

    if tmp:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    main()
