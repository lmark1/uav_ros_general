#!/usr/bin/env python

import pyproj
import sys

ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')

print '''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Paths</name>
    <description>Examples of paths. Note that the tessellate tag is by default
      set to 0. If you want to create tessellated lines, they must be authored
      (or edited) directly in KML.</description>
    <Style id="yellowLineGreenPoly">
      <LineStyle>
        <color>ff00aaff</color>
        <width>4</width>
      </LineStyle>
      <PolyStyle>
        <color>ff00aaff</color>
      </PolyStyle>
    </Style>
    <Placemark>
      <name>Absolute Extruded</name>
      <description>Transparent green wall with yellow outlines</description>
      <styleUrl>#yellowLineGreenPoly</styleUrl>
      <LineString>
        <extrude>0</extrude>
        <tessellate>1</tessellate>
        <altitudeMode>absolute</altitudeMode>
        <coordinates> '''
with open ('/home/ana/bags/prvipokusaj.txt') as f:
  for line in f:
    x, y, z = [float(f) for f in line.split(' ')]
    lon, lat, alt = pyproj.transform(ecef, lla, x, y, z, radians=False)
    print "{},{},{}".format(lon, lat, alt-45)


print '''</coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>'''



