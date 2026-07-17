#!/usr/bin/env python3

'''
This script fetches the latest version of the lua docs file from GitHub actions is passed a GitHub token.
Else pass in the downloaded .zip from the artifact or the extracted .md file.
It then converters the md to a rst, and does some editing
'''

import optparse
import os
import pathlib
import sys
import io

def download_artifact(token):
    import requests

    headers = {
        "Authorization": "token " + token,
        "User-Agent": "Python",
    }

    response = requests.get(
        "https://api.github.com/repos/ArduPilot/ardupilot/actions/artifacts",
        params={"name": "Docs"},
        headers=headers,
    )

    if response.status_code != 200:
        raise Exception("API request failed")

    data = response.json()

    download_url = None
    for artifact in data["artifacts"]:
        run = artifact["workflow_run"]
        if run["head_branch"] != "master":
            continue
        if artifact["expired"]:
            raise Exception("Artifact expired")
        download_url = artifact["archive_download_url"]
        break

    if download_url is None:
        raise Exception("No artifact found")

    # download
    lua_docs_name = os.path.join(os.getcwd(), 'docs.zip')
    with open(lua_docs_name, 'wb') as f:
        f.write(requests.get(download_url, headers=headers).content)

    return lua_docs_name

if __name__ == '__main__':

    parser = optparse.OptionParser(__file__)

    parser.add_option("--token",
                      type='string',
                      default=None,
                      help='GitHub token to allow auto downloading of artifact')

    opts, args = parser.parse_args()

    have_token = opts.token is not None

    if not have_token and (len(args) == 0):
        raise Exception("Need token or path")

    if have_token and (len(args) > 0):
        raise Exception("Need token or path, not both")

    docs = None
    if have_token:
        docs = download_artifact(opts.token)
    else:
        docs = args[0]

    docs = pathlib.Path(docs)
    if docs.suffix == '.zip':
        # Extract
        import shutil
        extract_to = (docs / "../scripting_docs").resolve()
        shutil.unpack_archive(docs, extract_to)
        docs = (extract_to / "ScriptingDocs.md").resolve()
        if not docs.exists():
            raise Exception("ScriptingDocs.md not found in extracted archive at: %s" % docs)

    if docs.suffix != '.md':
        raise Exception("Expected .md file got: %s" % docs)

    # Convert to rst
    try:
        from m2r2 import parse_from_file
        readme = parse_from_file(docs)
    except ImportError:
        print("Import m2r2 failed")
        print("Install with: python3 -m pip install m2r2")
        sys.exit(1)


    # Output converted to a file.
    rst = (pathlib.Path(__file__) / "../common/source/docs/ScriptingDocs.rst").resolve()
    with open(rst, 'w') as f:
        # Do find and replace on code blocks
        for line in io.StringIO(readme):
            if line.startswith(".. code-block:: lua"):
                line = line.replace(" lua", "")
            f.write(line)
