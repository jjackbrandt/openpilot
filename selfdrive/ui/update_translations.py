#!/usr/bin/env python3
import argparse
import json
import os
import pathlib
import xml.etree.ElementTree as ET

if "BASEDIR" in os.environ:
  BASEDIR = os.environ.get("BASEDIR")
else:
  from openpilot.common.basedir import BASEDIR

UI_DIR = os.path.join(BASEDIR, "selfdrive", "ui")
FROG_UI_DIR = os.path.join(BASEDIR, "selfdrive", "frogpilot", "ui")
TRANSLATIONS_DIR = os.path.join(UI_DIR, "translations")
LANGUAGES_FILE = os.path.join(TRANSLATIONS_DIR, "languages.json")
TRANSLATIONS_INCLUDE_FILE = os.path.join(TRANSLATIONS_DIR, "alerts_generated.h")
PLURAL_ONLY = ["main_en"]  # base language, only create entries for strings with plural forms


def generate_translations_include():
  # offroad alerts
  # TODO translate events from openpilot.selfdrive/controls/lib/events.py
  content = "// THIS IS AN AUTOGENERATED FILE, PLEASE EDIT alerts_offroad.json\n"
  with open(os.path.join(BASEDIR, "selfdrive/controls/lib/alerts_offroad.json")) as f:
    for alert in json.load(f).values():
      content += f'QT_TRANSLATE_NOOP("OffroadAlert", R"({alert["text"]})");\n'

  with open(TRANSLATIONS_INCLUDE_FILE, "w") as f:
    f.write(content)


def update_translations(vanish: bool = False, translation_files: None | list[str] = None, translations_dir: str = TRANSLATIONS_DIR):
  generate_translations_include()

  if translation_files is None:
    with open(LANGUAGES_FILE) as f:
      translation_files = json.load(f).values()

  for file in translation_files:
    tr_file = pathlib.Path(translations_dir) / f"{file}.ts"

    tree = ET.parse(tr_file)
    root = tree.getroot()

    backup = {}
    for context in root.findall("context"):
      name = context.find("name")
      if name is None:
        continue

      context_name = name.text
      for message in context.findall("message"):
        source = message.find("source")
        if source is None:
          continue

        source_text = source.text
        translation = message.find("translation")
        if translation is not None and "type" in translation.attrib:
          backup[(context_name, source_text)] = translation.attrib["type"]

    args = f"lupdate -locations none -recursive {UI_DIR} {FROG_UI_DIR} -ts {tr_file} -I {BASEDIR}"
    if vanish:
      args += " -no-obsolete"
    if file in PLURAL_ONLY:
      args += " -pluralonly"
    ret = os.system(args)
    assert ret == 0

    tree = ET.parse(tr_file)
    root = tree.getroot()

    for context in root.findall("context"):
      name = context.find("name")
      if name is None:
        continue

      context_name = name.text
      for message in context.findall("message"):
        source = message.find("source")
        if source is None:
          continue

        source_text = source.text
        if (context_name, source_text) in backup:
          translation = message.find("translation")
          if translation is not None:
            translation.attrib["type"] = backup[(context_name, source_text)]

    with tr_file.open("w", encoding="utf-8") as fp:
      fp.write('<?xml version="1.0" encoding="utf-8"?>\n' +
               '<!DOCTYPE TS>\n' +
               ET.tostring(root, encoding="utf-8", short_empty_elements=False).decode() +
               "\n")


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Update translation files for UI",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("--vanish", action="store_true", help="Remove translations with source text no longer found")
  args = parser.parse_args()

  update_translations(args.vanish)
