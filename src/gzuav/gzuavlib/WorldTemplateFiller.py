import math

class WorldTemplateFiller:
    def __init__(self, templateFilePath):
        with open(templateFilePath, 'rt') as templateStream:
            self._template = templateStream.read()
            self._uavModels = ''

    def add_uav(self, uav_type, uav_name, x, y, z, yaw):
        self._uavModels += '<model name="{}">\n'.format(uav_name)
        self._uavModels += '<include>\n'
        self._uavModels += '<uri>model://{}</uri>\n'.format(uav_type)
        self._uavModels += '<pose>{} {} {} 0 0 {}</pose>\n'.format(x, y, z, yaw * (math.pi / 180))
        self._uavModels += '</include>\n'
        self._uavModels += '</model>\n'

    def set_geo_ref(self, lat, lng, hgt, hdg):
        self._geoRef  = '<spherical_coordinates>\n'
        self._geoRef += '<latitude_deg>{}</latitude_deg>\n'.format(lat)
        self._geoRef += '<longitude_deg>{}</longitude_deg>\n'.format(lng)
        self._geoRef += '<elevation>{}</elevation>\n'.format(hgt)
        self._geoRef += '<heading_deg>{}</heading_deg>\n'.format(hdg)
        self._geoRef += '</spherical_coordinates>\n'

    def write_to(self, fp):
        contents = self._template \
            .replace('%SPHERICAL-COORDINATES-HERE%', self._geoRef, 1) \
            .replace('%UAV-MODELS-HERE%', self._uavModels, 1)

        fp.write(contents)
