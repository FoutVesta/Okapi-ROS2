cRoot = """<?xml version="1.0" ?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="%(robot_name)s">
    %(body)s
</robot>"""

cMaterials = """
<property name="M_PI" value="3.14159"/>
<property name="SCALE" value="0.0254"/>
<property name="base_x" value="0.55" />
<property name="base_y" value="0.55" />
<property name="base_radius_visual" value="0.2025" />
<property name="base_radius_collision" value="0.25" />  <!-- used to be 0.35 -->

<material name="Green">
      <color rgba="0.0 0.8 0.0 1.0"/>
</material>
<material name="Red">
      <color rgba="0.8 0.0 0.0 1.0"/>
</material>
<material name="White">
      <color rgba="1.0 1.0 1.0 1.0"/>
</material>
<material name="Blue">
      <color rgba="0.0 0.0 0.8 1.0"/>
</material>
<material name="Black">
      <color rgba="0.0 0.0 0.0 1.0"/>
</material>
<material name="Grey">
      <color rgba="0.25 0.25 0.25 1.0"/>
</material>
<material name="Silver">
      <color rgba="0.38 0.38 0.38 1.0"/>
</material>
<material name="Sandal">
      <color rgba="0.84 0.87 0.50 1.0"/>
</material>
<material name="LBlue">
      <color rgba="0.18 0.30 0.58 1.0"/>
</material>
<material name="Yellow">
      <color rgba="1 1 0 1.0"/>
</material>"""

cLinks = """

<link name="%(frame_name)s">
    <inertial>
        <mass value="0.00001" />
        <origin xyz="%(origin)s" />
        <inertia ixx="1.0" ixy="0.0" ixz="0.0"
                    iyy="1.0" iyz="0.0"
                    izz="1.0" />
    </inertial>

    <visual>
        <material name="%(vColor)s" />
        <!--origin xyz=" %(vOrigin)s " rpy="0 0 0" /-->
        <geometry>
            %(vGeometry)s
        </geometry>
    </visual>
</link>""" 


cJoins = """
<joint name="%(join_name)s" type="fixed">
    <origin xyz="%(lOrigin)s" rpy="0 0 0" />
    <parent link="%(parent_frame)s" />
    <child link="%(child_frame)s" />
</joint>"""

cBox = """<box size="%(vSize)s" />"""

cSphere = """<sphere radius="%(vRadius)s"/>"""

cHead = { 'robot_name': 'rfhHandheld' }

cItem = { 'vColor': 'Green', 'vRadius': '0.0254', 'vOrigin': '0 0 0', 'origin': '0 0 0', \
                 'vGeometry': 'cSphere', 'frame_name': 'ItemLabel'}

cBarcodeLabel = { 'vColor': 'White', 'vSize': '0.0127 0.05715 0.047625', 'vOrigin': '0 0 0', 'origin': '0 0 0', \
                 'vGeometry': 'cBox', 'frame_name': 'barcodeLabel'}

cBarcode = { 'vColor': 'Black', 'vSize': '0.00125 0.0005 0.0125', 'vOrigin': '0 0 0', 'origin': '0 0 0', \
                 'vGeometry': 'cBox', 'frame_name': 'barcode' }

cRack = { 'vColor': 'LBlue', 'vSize': '1.0 48.0 72.0', 'vOrigin': '0 0 0', 'origin': '0 0 0', \
                 'vGeometry': 'cBox', 'frame_name': 'rack' }

cLevel = { 'vColor': 'LBlue', 'vSize': '19.5 48.0 1.0', 'vOrigin': '0 0 0', 'origin': '0 0 0', \
                 'vGeometry': 'cBox', 'frame_name': 'level' }

cDefault= { 'vColor': 'Red', 'vSize': '0.025 0.025 0.025', 'vOrigin': '0 0 0', 'origin': '0 0 0', \
                 'vGeometry': 'cBox', 'frame_name': 'global' }

cConnect = { 'join_name': 'join', 'lOrigin': '0 1 0', 'parent_frame': 'parent', 'child_frame': 'child' }

cRackConfig = { 'T1': { 'vRSize': [1.0, 192.0, 78.0], 'vLSize': [19.5, 192.0, 1.0], 'vLevels': 5 }, \
                'T2': { 'vRSize': [0.0025, 0.25, 0.5], 'vLSize': [0.125, 0.25, 0.0025], 'vLevels': 3 }, \
                'T3': { 'vRSize': [0.0025, 0.25, 0.5], 'vLSize': [0.250, 0.25, 0.0025], 'vLevels': 3 }, \
                'T4': { 'vRSize': [0.0254, 4.8768, 1.9812], 'vLSize': [0.4953, 4.8768, 0.0254], 'vLevels': [0.1397, 0.5334, 0.9779, 1.4351, 1.8161] }, \
                'T5': { 'vRSize': [1.0, 192.0, 78.0], 'vLSize': [19.5, 192.0, 1.0], 'vLevels': [5.5,21.0,38.5,56.5,71.5] } }

cDoublesided = ('T3',)

cVariableSpaced = ('T4', 'T5')

cRackWallBarcode = ('T2','T5',)

cRackFloatBarcode = ('T4',)

cMargin = 0.1

cGlobalFlag = 1


if __name__ == '__main__':
    body = cMaterials
    if cBarcodeLabel['vGeometry']=='cBox':
       cBarcodeLabel['vGeometry'] = cBox%cBarcodeLabel
    body += cLinks%cBarcodeLabel
    body += cJoins%cConnect
    cHead['body'] = body
    print cRoot%cHead
