# ! /usr/bin/env python

from __future__ import print_function

import codecs
import os
import warnings
import yaml
from collections import defaultdict, deque, Iterable

import rospy
from mbot_robot_class_ros import mbot as mbot_class
import numpy as np


def dict_zip(a, b):
    function_type = type(lambda: None)

    if isinstance(a, Iterable) and isinstance(b, function_type):
        return dict(zip(a, map(b, a)))

    elif isinstance(a, function_type) and isinstance(b, Iterable):
        return dict(zip(map(a, b), b))

    elif isinstance(a, Iterable) and isinstance(b, Iterable):
        if not len(a) == len(b):
            raise Exception

        return dict(zip(a, b))

    else:
        raise TypeError


class SemanticEntity(object):
    def __init__(self, uuid, semantic_name, semantic_type):
        assert('__eq__' in dir(uuid))
        assert('__hash__' in dir(uuid))
        assert('__eq__' in dir(semantic_name))
        assert('__str__' in dir(semantic_name))

        self._uuid = uuid
        self._semantic_name = semantic_name
        self._semantic_type = semantic_type

    def _add_contained_entity(self, entity):
        pass

    def _remove_contained_entity(self, entity):
        pass

    @property
    def uuid(self):
        """
        Universally unique identifier.
        Every semantic entity has this property.
        """
        return self._uuid

    @property
    def semantic_name(self):
        """
        The name associated to this semantic entity.
        It does not need to be unique, e.g., two objects can have orange as name.
        Every semantic entity has this property.
        """
        return self._semantic_name

    @property
    def semantic_type(self):
        """
        The type associated to this semantic entity.
        It does not need to be unique, e.g., two objects can have known_object as type.
        Every semantic entity has this property.
        """
        return self._semantic_type

    @property
    def tree_path(self):
        raise NotImplementedError

    @property
    def tree_path_str(self):
        assert(isinstance(self.tree_path, Iterable))
        return reduce(os.path.join, self.tree_path, '/')

    def __str__(self):
        return self._semantic_name.__str__()

    def __eq__(self, other):
        return isinstance(self, type(other)) and self.uuid.__eq__(other.uuid)

    def __hash__(self):
        return hash((type(self), self.uuid))


class SemanticMap(SemanticEntity):

    def __init__(self, uuid, semantic_name, semantic_type, contains_rooms=None, properties=None):
        super(SemanticMap, self).__init__(uuid, semantic_name, semantic_type)
        self._rooms = dict()

        if contains_rooms is not None:
            for room in contains_rooms:
                assert(isinstance(room, Room))
                room.in_map = self

        self.properties = properties

    @staticmethod
    def load(semantic_map_path):
        """
        Loads a semantic map from a yaml file.
        :param semantic_map_path: the path of a yaml file containing the description of a semantic map.
        :raises IOError if the file does not exists. yaml.YAMLError if the yaml file is not well formed.
        :return a SemanticMap object, initialised with values from the yaml file.
        """

        with codecs.open(os.path.expanduser(semantic_map_path), 'r', 'utf8') as f:
            smf = yaml.load(f)

        map_yaml_properties = smf['map']['properties'] if 'properties' in smf['map'] else None

        map_ = SemanticMap(uuid=smf['map']['uuid'],
                           semantic_name=smf['map']['semantic_name'],
                           semantic_type=smf['map']['semantic_type'],
                           properties=map_yaml_properties)  # type: SemanticMap

        if 'rooms' in smf:
            for room_uuid, room_dict in smf['rooms'].iteritems():
                yaml_properties = room_dict['properties'] if 'properties' in room_dict else None
                Room(uuid=room_uuid,
                     semantic_name=room_dict['semantic_name'],
                     semantic_type=room_dict['semantic_type'],
                     in_map=map_,
                     properties=yaml_properties)

            for room_uuid, room_dict in smf['rooms'].iteritems():
                for connected_uuid in room_dict['connected_to']:
                    room_1 = map_.get_room(room_uuid)
                    room_2 = map_.get_room(connected_uuid)
                    if room_1 is None:
                        raise KeyError('room with uuid %s not found' % room_uuid)
                    if room_2 is None:
                        raise KeyError('room with uuid %s not found' % connected_uuid)
                    room_1.add_connection(room_2)

        if 'locations' in smf:
            for location_uuid, location_dict in smf['locations'].iteritems():
                room = map_.get_room(location_dict['in_room'])
                yaml_properties = location_dict['properties'] if 'properties' in location_dict else None
                if room is None:
                    raise KeyError('room with uuid %s not found' % location_dict['in_room'])
                Location(uuid=location_uuid,
                         semantic_name=location_dict['semantic_name'],
                         semantic_type=location_dict['semantic_type'],
                         in_room=room,
                         properties=yaml_properties)

        if 'objects' in smf:
            for obj_uuid, obj_dict in smf['objects'].iteritems():
                location = map_.get_location(obj_dict['at_location'])
                yaml_properties = obj_dict['properties'] if 'properties' in obj_dict else None
                if location is None:
                    raise KeyError('location with uuid %s not found' % obj_dict['at_location'])
                Object(uuid=obj_uuid,
                       semantic_name=obj_dict['semantic_name'],
                       semantic_type=obj_dict['semantic_type'],
                       at_location=location,
                       properties=yaml_properties)

        return map_

    def dump(self, semantic_map_path):
        """
        Dumps the map to a yaml file.
        """
        raise NotImplemented  # TODO

    def copy(self):
        raise NotImplemented  # TODO

    @property
    def contains_rooms(self):
        """
        The list of rooms contained in the semantic map.
        """
        return self._rooms.values()

    @property
    def rooms_by_uuid(self):
        """
        The dict of rooms contained in the semantic map. Keys are the rooms' uuid.
        """
        return self._rooms

    def get_room(self, room_uuid):
        """
        :returns: the room with given uuid if such room exists in the semantic map, otherwise None.
        """
        return self._rooms[room_uuid] if room_uuid in self._rooms.keys() else None

    @property
    def contains_locations(self):
        """
        The list of all locations contained in the semantic map.
        """
        locations = list()
        for room in self.contains_rooms:
            locations += room.contains_locations
        return locations

    def get_location(self, location_uuid):
        """
        :returns: the location with given uuid if such location exists in the semantic map, otherwise None.
        """
        for room in self.contains_rooms:  # type: Room
            if location_uuid in room.locations_by_uuid.keys():
                return room.locations_by_uuid[location_uuid]
        return None

    @property
    def contains_objects(self):
        """
        The list of all objects contained in the semantic map.
        """
        objects = list()
        for location in self.contains_locations:
            objects += location.contains_objects
        return objects

    def get_object(self, obj_uuid):
        """
        :returns: the object with given uuid if such object exists in the semantic map, otherwise None.
        """
        for room in self.contains_rooms:  # type: Room
            for location in room.contains_locations:  # type: Location
                if obj_uuid in location.objects_by_uuid.keys():
                    return location.objects_by_uuid[obj_uuid]
        return None

    def get_objects_by_name(self, name):
        """
        :returns: the list of all objects with given semantic name contained in the semantic map.
        """
        return filter(lambda obj: obj.semantic_name == name, self.contains_objects)

    def _add_contained_entity(self, room):
        assert(isinstance(room, Room))
        self._rooms[room.uuid] = room

    def _remove_contained_entity(self, room):
        assert(isinstance(room, Room))

        try:
            del self._rooms[room.uuid]
        except KeyError:
            warnings.warn("tried to remove room [%s] from map [%s] but room not in map" % (str(room), str(self)))

    @property
    def tree_path(self):
        return [str(self.uuid)]

    @staticmethod
    def tree_path_str(tree_path):
        return reduce(os.path.join, tree_path, '/')

    def __sub__(self, other):
        assert(isinstance(other, type(self)))

        objects = self.contains_objects
        other_objects = other.contains_objects

        objects_by_path = dict_zip(lambda obj: obj.tree_path_str, objects)
        other_objects_by_path = dict_zip(lambda obj: obj.tree_path_str, other_objects)

        objects_set = set(objects_by_path.keys())
        other_objects_set = set(other_objects_by_path.keys())

        result_set = objects_set - other_objects_set
        result = map(lambda obj_path: objects_by_path[obj_path], result_set)
        return result

    def intersection(self, other):
        assert(isinstance(other, type(self)))

        objects = self.contains_objects
        other_objects = other.contains_objects

        objects_by_path = dict_zip(lambda obj: obj.tree_path_str, objects)
        other_objects_by_path = dict_zip(lambda obj: obj.tree_path_str, other_objects)

        objects_set = set(objects_by_path.keys())
        other_objects_set = set(other_objects_by_path.keys())

        result_set = objects_set.intersection(other_objects_set)
        result = map(lambda obj_path: objects_by_path[obj_path], result_set)
        return result

    def symmetric_difference(self, other):
        assert(isinstance(other, type(self)))

        objects = self.contains_objects
        other_objects = other.contains_objects

        objects_by_path = dict_zip(lambda obj: obj.tree_path_str, objects)
        other_objects_by_path = dict_zip(lambda obj: obj.tree_path_str, other_objects)

        objects_set = set(objects_by_path.keys())
        other_objects_set = set(other_objects_by_path.keys())

        result_set = objects_set - other_objects_set
        other_result_set = other_objects_set - objects_set
        result = map(lambda obj_path: objects_by_path[obj_path], result_set)
        result += map(lambda obj_path: other_objects_by_path[obj_path], other_result_set)
        return result

    def print_tree(self):

        print(self)
        for room in self.contains_rooms:
            print('\t', room, '(connected to: %s)' % ', '.join(map(str, room.connected_to)))
            for location in room.contains_locations:
                print('\t\t', location)
                for obj in location.contains_objects:
                    print('\t\t\t', obj)


class Room(SemanticEntity):
    """
        relations:
            in_map SemanticMap,
            contains_locations {SemanticMap.Location}
            connected_to {SemanticMap.Room},
    """
    _map = None  # type: SemanticEntity

    def __init__(self, uuid, semantic_name, semantic_type, contains_locations=None, connected_to=None, in_map=None, properties=None):
        super(Room, self).__init__(uuid, semantic_name, semantic_type)
        assert(in_map is None or isinstance(in_map, SemanticMap))

        self._locations = dict()
        self._connected_to = set()
        self._map = None

        if contains_locations is not None:
            for location in contains_locations:
                assert(isinstance(location, Location))
                location.in_room = self

        if connected_to is not None:
            for room in connected_to:
                assert(isinstance(room, Room))
                room.add_connection(self)

        self.in_map = in_map
        self.properties = properties

    def copy(self):
        raise NotImplemented  # TODO

    @property
    def in_map(self):
        return self._map

    @in_map.setter
    def in_map(self, mapp):
        assert(mapp is None or isinstance(mapp, SemanticMap))

        if self._map is not None:
            self._map._remove_contained_entity(self)

        self._map = mapp

        if self._map is not None:
            self._map._add_contained_entity(self)

    @property
    def contains_locations(self):
        return self._locations.values()

    @property
    def locations_by_uuid(self):
        """
        The dict of locations contained in this room. Keys are the locations' uuid.
        """
        return self._locations

    def _add_contained_entity(self, location):
        assert(isinstance(location, Location))
        self._locations[location.uuid] = location

    def _remove_contained_entity(self, location):
        assert(isinstance(location, Location))

        try:
            del self._locations[location.uuid]
            location.in_room = None
        except KeyError:
            warnings.warn("tried to remove location [%s] from room [%s],"
                          " but location not in room" % (str(location), str(self)))

    @property
    def connected_to(self):
        return self._connected_to

    def add_connection(self, room):
        assert(isinstance(room, Room))

        self._connected_to.add(room)
        room._connected_to.add(self)

    def remove_connection(self, room):
        assert(isinstance(room, Room))

        try:
            self._connected_to.remove(room)
        except KeyError:
            warnings.warn("tried to remove connection from room [%s] to room [%s],"
                          " but room not in connected rooms" % (str(self), str(room)))

        try:
            room._connected_to.remove(self)
        except KeyError:
            warnings.warn("tried to remove connection from room [%s] to room [%s],"
                          " but room not in connected rooms" % (str(room), str(self)))

    @property
    def tree_path(self):
        if self.in_map is not None:
            return self.in_map.tree_path + [str(self.uuid)]
        else:
            return [str(self.uuid)]


class Location(SemanticEntity):
    """
        relations:
            in_room SemanticMap.Room,
            contains_objects {SemanticMap.Object}
    """
    _room = None  # type: SemanticEntity

    def __init__(self, uuid, semantic_name, semantic_type, contains_objects=None, in_room=None, properties=None):
        super(Location, self).__init__(uuid, semantic_name, semantic_type)
        assert(in_room is None or isinstance(in_room, Room))

        self._objects = dict()
        self._room = None

        if contains_objects is not None:
            for obj in contains_objects:
                assert(isinstance(obj, Object))
                obj.at_location = self

        self.in_room = in_room
        self.properties = properties

    def copy(self):
        raise NotImplemented  # TODO

    @property
    def in_room(self):
        return self._room

    @in_room.setter
    def in_room(self, room):
        assert(room is None or isinstance(room, Room))

        if self._room is not None:
            self._room._remove_contained_entity(self)

        self._room = room

        if room is not None:
            self._room._add_contained_entity(self)

    @property
    def contains_objects(self):
        return self._objects.values()

    @property
    def objects_by_uuid(self):
        """
        The dict of objects contained in this room. Keys are the objects' uuid.
        """
        return self._objects

    def _add_contained_entity(self, obj):
        assert(isinstance(obj, Object))
        self._objects[obj.uuid] = obj

    def _remove_contained_entity(self, obj):
        assert(isinstance(obj, Object))

        try:
            del self._objects[obj.uuid]
        except KeyError:
            warnings.warn("tried to remove object [%s] from location [%s],"
                          " but object not in location" % (str(obj), str(self)))

    @property
    def tree_path(self):
        if self.in_room is not None:
            return self.in_room.tree_path + [str(self.uuid)]
        else:
            return [str(self.uuid)]


class Object(SemanticEntity):
    """
        relations: at_location SemanticMap.Location
    """
    _location = None  # type: SemanticEntity

    def __init__(self, uuid, semantic_name, semantic_type, at_location=None, properties=None):
        super(Object, self).__init__(uuid, semantic_name, semantic_type)
        assert(at_location is None or isinstance(at_location, Location))

        self._location = None

        self.at_location = at_location
        self.properties = properties

    def copy(self):
        raise NotImplemented  # TODO

    @property
    def at_location(self):
        return self._location

    @at_location.setter
    def at_location(self, location):
        assert(location is None or isinstance(location, Location))

        if self._location is not None:
            self._location._remove_contained_entity(self)

        self._location = location

        if location is not None:
            self._location._add_contained_entity(self)

    @property
    def tree_path(self):
        if self.at_location is not None:
            return self.at_location.tree_path + [str(self.uuid)]
        else:
            return [str(self.uuid)]


class SemanticMapping(object):

    waypoints_visit_list = None  # type: deque
    map = None  # type: SemanticMap

    def __init__(self):

        self.mbot = mbot_class.mbotRobot(enabled_components=["hri", "navigation", "yolo", "perception"])
        world_model_path = rospy.get_param('~world_model_path')
        self.object_to_location_max_distance = rospy.get_param('~object_to_location_max_distance', None)

        # check semantic map file is available
        semantic_map_filename = os.path.join(world_model_path, 'semantic_map.yaml')
        self.try_load_yaml(semantic_map_filename, logfatal="semantic_map.yaml not found in %s" % world_model_path)

        # load default semantic map
        self.default_map = SemanticMap.load(semantic_map_filename)
        print('Default semantic map:')
        self.default_map.print_tree()

        support_surfaces_filename = os.path.join(world_model_path, 'support_surface_polygons.yaml')

        self.mbot.perception.init_semantic_map(semantic_map_filename, support_surfaces_filename, self.object_to_location_max_distance)

        # get the semantic map
        self.map = self.mbot.perception.get_semantic_map()

        print('Initial semantic map:')
        self.map.print_tree()

        # load and compute location gaze points from support surfaces
        self.location_uuid_to_gaze_target_dict = dict()
        ssp_filename = os.path.join(world_model_path, 'support_surface_polygons.yaml')
        polygons = self.try_load_yaml(ssp_filename,
                                      logfatal="support_surface_polygons.yaml not found in %s" % world_model_path)

        for location in self.map.contains_locations:
            if 'support_surface_polygon' in location.properties:
                support_surface_polygon = polygons['support_surfaces'][location.properties['support_surface_polygon']]
                support_surface_point = map(float, np.average(np.array(support_surface_polygon), axis=0))
                self.location_uuid_to_gaze_target_dict[location.uuid] = support_surface_point

        # load and compute the waypoint visit list
        waypoints_visit_list_filename = os.path.join(world_model_path, 'waypoints_visit_list.yaml')
        waypoints_visit_list = self.try_load_yaml(waypoints_visit_list_filename,
                                                  logfatal="waypoints_visit_list.yaml not found in %s" % world_model_path)

        self.waypoints_visit_list = deque(waypoints_visit_list['semantic_mapping'])

        self.waypoint_to_locations_dict = defaultdict(list)
        for location in self.map.contains_locations:
            if 'to_be_explored' in location.properties \
                    and location.properties['to_be_explored'] \
                    and 'exploration_waypoints' in location.properties:

                for waypoint in location.properties['exploration_waypoints']:
                    self.waypoint_to_locations_dict[waypoint].append(location)

        # log warnings and errors for mismatch in waypoint parameters
        waypoints_from_locations = set(self.waypoint_to_locations_dict.keys())
        waypoints_from_visit_list = set(self.waypoints_visit_list)
        available_navigation_waypoints = set(self.mbot.navigation.get_available_locations())

        if len(waypoints_from_locations - waypoints_from_visit_list) > 0:
            rospy.logwarn("Exploration waypoints defined in the semantic map locations, "
                          "but not used in the visit list: \n%s" %
                          ', '.join(list(waypoints_from_locations - waypoints_from_visit_list)))

        if len(waypoints_from_visit_list - waypoints_from_locations) > 0:
            rospy.logerr("Exploration waypoints in the visit list, "
                         "but not defined in the semantic map locations: \n%s" %
                         ', '.join(list(waypoints_from_visit_list - waypoints_from_locations)))

        if len(waypoints_from_visit_list - available_navigation_waypoints) > 0:
            rospy.logerr("Exploration waypoints in the visit list, "
                         "but not available for navigation: \n%s" %
                         ', '.join(list(waypoints_from_visit_list - available_navigation_waypoints)))

    def loop(self):

        rospy.sleep(2.0)  # wait to make sure nodes are ready to receive their start events

        self.mbot.yolo.small_objects_tracking(True)
        self.mbot.yolo.people_tracking(True)

        for current_waypoint in self.waypoints_visit_list:

            for current_location in self.waypoint_to_locations_dict[current_waypoint]:

                if rospy.is_shutdown():
                    return

                if current_waypoint not in self.mbot.navigation.get_available_locations():
                    rospy.logerr("could not navigate to %s. Waypoint not available" % current_waypoint)
                    continue

                # start gazing toward the current location
                location_gaze_target = self.location_uuid_to_gaze_target_dict[current_location.uuid]
                gaze_successful = self.mbot.perception.gaze_once(location_gaze_target)
                if not gaze_successful:
                    rospy.logwarn('not able to gaze toward location %s (%s)' %
                                  (str(current_location), location_gaze_target))

                if rospy.is_shutdown():
                    return

                self.mbot.perception.gaze_start(location_gaze_target)

                # navigate to the waypoint of the current location
                navigation_successful = self.mbot.navigation.go_to_location(current_waypoint,
                                                                            timeout=60.0, use_gazing=False,
                                                                            # navigation_config='front_and_back_slow',
                                                                            navigation_config='front_and_back',
                                                                            dwa_config='rough')

                # wait one second for perception
                rospy.sleep(3.0)
                self.mbot.perception.gaze_stop()

                self.mbot.perception.update_semantic_map()

                # print a nice list of locations and objects
                print("\nUpdated semantic map after last waypoint:")
                for l in self.map.contains_locations:
                    print(l)
                    for o in l.contains_objects:
                        print("\t", o)

                if not navigation_successful:
                    rospy.logwarn("could not reach %s." % current_waypoint)
                    continue

                if rospy.is_shutdown():
                    return

        # stop tracking
        self.mbot.yolo.small_objects_tracking(False)
        self.mbot.yolo.people_tracking(False)

        self.mbot.hri.say_and_wait("Finished.")
        print('Final semantic map:')
        self.map.print_tree()

    @staticmethod
    def try_load_yaml(filename, logfatal):
        if not os.path.isfile(filename):
            rospy.logfatal(logfatal)
            rospy.signal_shutdown(logfatal)

        with codecs.open(filename, 'r', 'utf8') as f:
            loaded_yaml = yaml.load(f)

        return loaded_yaml


def print_path_list(objects_iterable):
    assert(isinstance(objects_iterable, Iterable))
    assert(all(map(lambda obj: isinstance(obj, Object), objects_iterable)))

    print('\n'.join(map(lambda obj: obj.tree_path_str, objects_iterable)))


if __name__ == '__main__':  # TODO: make an actual test unit
    # some testing, only executed when calling the node directly

    m1 = SemanticMap.load("/home/enrico/ros_ws/src/isr_monarch_robot/mbot_world_model/maps/isr-lab/semantic_map_gt.yaml")
    print('m1:')
    m1.print_tree()

    m2 = SemanticMap.load("/home/enrico/ros_ws/src/isr_monarch_robot/mbot_world_model/maps/isr-lab/semantic_map_gt.yaml")
    print('m2:')
    m2.get_object('toilet_paper').at_location = None
    Object(uuid='test_object',
           semantic_name='test_object_name',
           semantic_type='known_object',
           at_location=m2.get_location('kitchen_table_location'))
    m2.print_tree()

    # test: sub operator
    print("\n m1 - m2:")
    print_path_list(m1 - m2)

    print("\n m2 - m1:")
    print_path_list(m2 - m1)

    # test: intersection operator
    print("\n m1.intersection(m2):")
    print_path_list(m1.intersection(m2))

    # test: symmetric_difference operator
    print("\n m1.symmetric_difference(m2):")
    print_path_list(m1.symmetric_difference(m2))

    print("\n m2.symmetric_difference(m1):")
    print_path_list(m2.symmetric_difference(m1))

    # test: get_objects_by_name
    print("\n cokes:")
    print_path_list(m1.get_objects_by_name('coke'))

    # test: clearing location's objects
    kitchen_table = m2.get_location('kitchen_table_location')
    print(kitchen_table, ':')
    map(print, kitchen_table.contains_objects)

    for obj_ in kitchen_table.contains_objects:
        obj_.at_location = None

    kitchen_table = m2.get_location('kitchen_table_location')
    print(kitchen_table, ':')
    map(print, kitchen_table.contains_objects)
