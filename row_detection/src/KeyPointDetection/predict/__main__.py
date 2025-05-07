#!/usr/bin/env python3

"""
Test/Infer using the point detection network
"""

import argparse
from lxml import etree
from .exPointDetection.testing import WebHandler, NetworkTester


def getOptions():
    """
    Command line options parser
    """

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f", "--filestem", dest="filestem", help="Filestem of the keras model to use"
    )
    parser.add_argument(
        "-V",
        "--visualization",
        dest="visualization",
        action="store_true",
        help="Create a visualization of the point",
    )
    parser.add_argument(
        "-d",
        "--daemon",
        dest="daemon",
        action="store_true",
        help="run as daemon (web server)",
    )
    parser.add_argument(
        "--xml-list",
        dest="xmlList",
        type=str,
        default=None,
        help="File containing list of XMLs for testing",
    )
    parser.add_argument(
        "--output-type",
        dest="outputType",
        type=str,
        default="pixel",
        help="Output value unit - Options: [pixel, ratio] - Image pixel values or (0-1)",
    )
    parser.add_argument(
        "-p",
        "--port",
        type=int,
        default=8080,
        help="when running as daemon, listen at this port number",
    )
    parser.add_argument("images", type=str, nargs="*", help="Images to predict")

    return parser.parse_args()


def main():
    """
    main()
    """

    # Get the options
    options = getOptions()

    # Set the webserver port to match the options
    WebHandler.port = options.port

    nt = NetworkTester()
    if options.filestem:
        nt.filestem = options.filestem
    nt.visualization = options.visualization
    nt.outputType = options.outputType

    if options.xmlList:
        with open(options.xmlList, "r") as file:
            lines = file.readlines()
            for xmlPath in lines:
                tree = etree.parse(xmlPath.strip())
                imagePath = tree.find("filename").text
                options.images.append(imagePath)
    nt.run(filenames=options.images, server=options.daemon)


if __name__ == "__main__":
    main()
