import os
import argparse
import sys


class Extractor():

    def launch(self, argv):
        self.parse_arg(argv)
        return 'Parsed the arguments: Node ' + self.args.name + ' Package:' + self.args.package_name


    def parse_arg(self, argv):
        parser = argparse.ArgumentParser()
        mutually_exclusive = parser.add_mutually_exclusive_group()
        mutually_exclusive.add_argument(
            '--node', '-n', help="node analyse", action='store_true')
        mutually_exclusive.add_argument(
            '--launch', '-l', help="launch analyse", action='store_true')
        parser.add_argument('--package', required=True, dest='package_name')
        parser.add_argument('--name', required=True, dest='name')
        self.args = parser.parse_args(args=argv)


def main(argv=None):
    extractor = Extractor()
    return extractor.launch(argv)


if __name__ == "__main__":
    main()
