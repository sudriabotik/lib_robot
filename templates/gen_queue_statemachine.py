#!/bin/env python3

import sys
import re


def usage() :

	print(f"usage : {sys.argv[0]} <definition_file> <output_folder_inc> <output_folder_src>")


if len(sys.argv) < 4 :
	usage()
elif sys.argv[1] == "help" :
	usage()


# read the definition file

re.compile("(// +[A-Z_])\n")