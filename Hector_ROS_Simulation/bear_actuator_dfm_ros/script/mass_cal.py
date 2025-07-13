#!/usr/bin/env python3
import xml.etree.ElementTree as ET
# URDFファイルを読み込む
urdf_file = 'wwlambda_r2.urdf'  # ←URDFファイル名を指定
tree = ET.parse(urdf_file)
root = tree.getroot()

total_mass = 0.0
print(root)
# URDF内の全ての<inertial><mass>タグを探す
for link in root.findall('link'):
    inertial = link.find('inertial')
    if inertial is not None:
        mass_tag = inertial.find('mass')
        if mass_tag is not None:
            mass = float(mass_tag.attrib['value'])
            total_mass += mass
            print(f"Link {link.attrib['name']}: mass = {mass}")
        else:
            print(f"Link {link.attrib['name']} has no mass tag!")
    else:
        print(f"Link {link.attrib['name']} has no inertial tag!")

print(f"Total robot mass: {total_mass} kg")
