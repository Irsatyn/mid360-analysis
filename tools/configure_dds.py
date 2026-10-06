#!/usr/bin/env python3
"""Generate a CycloneDDS configuration for a host/interface and optional peers."""
import argparse
import ipaddress
from pathlib import Path
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='auto', help='auto, interface name, or local IPv4')
    parser.add_argument('--peer', action='append', default=[], help='remote ROS host IPv4; repeat for multiple hosts')
    parser.add_argument('--multicast', choices=['spdp', 'true', 'false'], default='spdp')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    for peer in args.peer:
        try:
            address = ipaddress.IPv4Address(peer)
            if address.is_multicast or address.is_unspecified or address.is_reserved:
                raise ValueError(peer)
        except ValueError:
            parser.error(f'peer must be a unicast IPv4 address: {peer!r}')
    ns = 'https://cdds.io/config'
    ET.register_namespace('', ns)
    template = Path(__file__).resolve().parents[1] / 'mid360_analysis/config/cyclonedds.xml'
    tree = ET.parse(template)
    interface = tree.find(f'.//{{{ns}}}NetworkInterface')
    interface.attrib.clear()
    if args.interface == 'auto':
        interface.set('autodetermine', 'true')
    else:
        try:
            address = ipaddress.IPv4Address(args.interface)
        except ValueError:
            interface.set('name', args.interface)
        else:
            interface.set('address', str(address))
    tree.find(f'.//{{{ns}}}AllowMulticast').text = args.multicast
    peers = tree.find(f'.//{{{ns}}}Peers')
    for peer in args.peer:
        ET.SubElement(peers, f'{{{ns}}}Peer', Address=peer)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output, encoding='utf-8', xml_declaration=True)
    print(args.output.resolve().as_uri())


if __name__ == '__main__':
    main()
