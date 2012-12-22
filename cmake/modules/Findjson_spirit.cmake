find_file(json_spirit_FILE json_spirit.h
    HINTS
    /usr/include
    )

mark_as_advanced(json_spirit_FILE)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(json_spirit
  "Could not find json-spirit. On ubuntu please:
apt-get install libjson-spirit-dev
"
json_spirit_FILE)
get_filename_component(JSON_SPIRIT_INCLUDE_DIR ${json_spirit_FILE} PATH)

#no headers required
