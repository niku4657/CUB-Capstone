#!/usr/bin/python3

import argparse
from queue import Queue, Empty
from threading import Thread
from multiprocessing import cpu_count

import pybktree
from PIL import Image
import imagehash


def getOptions():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-u",
        "--unique",
        dest="printUnique",
        action="store_true",
        help="Do not print duplicates, instead print unique filenames",
    )
    parser.add_argument(
        "-d",
        "--min-distance",
        default=4,
        type=int,
        dest="minDistance",
        help="Minimum hamming distance to qualify as unique",
    )
    parser.add_argument(
        "--hashing-algorithm",
        default="phash",
        choices=["phash", "whash", "dhash", "average_hash"],
        dest="hashingAlgorithm",
        help="Image hashing algorithm to run on image",
    )
    parser.add_argument(
        "--image-list",
        default=None,
        dest="imageList",
        help="List of images to process, one line per filename",
    )
    parser.add_argument(
        "--deterministic",
        action="store_true",
        help="Run in deterministic mode, using only one reader thread",
    )
    parser.add_argument("filenames", help="Image filenames to process", nargs="*")

    options = parser.parse_args()
    options.hashingAlgorithm = getattr(imagehash, options.hashingAlgorithm)
    if options.imageList:
        options.filenames = [line.rstrip() for line in open(options.imageList)]
    if options.deterministic:
        options.numReaderThreads = 1
    else:
        options.numReaderThreads = cpu_count() - 1

    return options


def itemDistance(hash1, hash2):
    return hash1 - hash2


def hashCalcThread(q, alg, hq):
    while True:
        try:
            fn = q.get(timeout=0.1)
            hq.put((fn, alg(Image.open(fn))))
            q.task_done()
        except Empty:
            if q.keepGoing:
                continue
            else:
                return


def hashComparisonThread(hq, options):

    tree = pybktree.BKTree(itemDistance)
    while True:
        try:
            fn, h = hq.get(timeout=0.1)
            if not tree.find(h, options.minDistance):
                tree.add(h)
                if options.printUnique:
                    print(fn)
            else:
                if not options.printUnique:
                    print(fn)
            hq.task_done()
        except Empty:
            if hq.keepGoing:
                continue
            else:
                return


def main():

    options = getOptions()

    # Allocate our structures
    threads = list()
    q = Queue(maxsize=1000)
    q.keepGoing = True
    hq = Queue(maxsize=1000)
    hq.keepGoing = True

    # Launch threads to handle the q
    for _ in range(0, options.numReaderThreads):
        threads.append(
            Thread(target=hashCalcThread, args=(q, options.hashingAlgorithm, hq))
        )
        threads[-1].start()

    # Launch thread to handle the hash queue
    ht = Thread(target=hashComparisonThread, args=(hq, options))
    ht.start()

    # Dump all the filenames to process in a queue
    for fn in options.filenames:
        q.put(fn)

    # Notify the threads that they can quit once the queue becomes empty
    q.keepGoing = False

    # Wait for all the queue loading threads to complete
    for t in threads:
        t.join()

    # Notify the tree thread that it can quit once the queue becomes empty
    hq.keepGoing = False

    ht.join()


if __name__ == "__main__":
    main()
