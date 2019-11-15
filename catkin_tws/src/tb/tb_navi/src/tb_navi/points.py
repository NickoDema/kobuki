#!/usr/bin/env python

##   agv_workspace.py
##   Created on: 11.09.2018
##           By: Nikoaly Dema
##        Email: ndema2301@gmail.com
##               goTRG | spb

import os, json
import rospkg

from datetime import datetime

class PointsHolder():
    def __init__(self, locs_db_name = 'main'):

        self.locs = {}

        # get path to locs dir
        self.locs_db_name = locs_db_name
        rospack = rospkg.RosPack()
        self.locs_dpath = rospack.get_path('agv_navigation') + "/points/"
        # init load locs
        self.locs = self.load_locs_db(locs_db_name)
        #print(json.dumps(self.locs, indent=2))
        #self.new_db("test")
        del rospack


    # try to find locs on local machine
    # if in locs dir there is no specified file return False
    def load_locs_db(self, locs_db_name, set_current_locs_db = False):
        locs_fpath = self.locs_dpath + locs_db_name + '.locs'
        #print(locs_fpath)
        if os.path.isfile(locs_fpath):
            try:
                locs_file = open(locs_fpath,'r')
            except:
                print('[AGV]: File ' + locs_db_name + ' is exist, but ' +
                      'can\'t be opened for some reason')
                return None

            try:
                locs_db = json.load(locs_file, encoding='utf-8')
            except:
                print('[AGV]: File ' + locs_db_name + ' can\'t ' +
                      'be processed by json parser')
                locs_file.close()
                return None

            locs_file.close()
            #print(json.dumps(locs_db, indent=2))
            if set_current_locs_db:
                self.locs = locs_db
            return locs_db
            #print(json.dumps(locs_db, indent=2))#, sort_keys=True))
            #return True
        else:
            print('[AGV]: File ' + locs_db_name + ' does not exist')
            return None


    # create new local locs database
    # return False if file is already exist
    def new_db(self, db_name, db_type = "locs", data = {}, rewrite = False):
        locs_fpath = self.locs_dpath + db_name + '.' + db_type
        if os.path.isfile(locs_fpath) and not rewrite:
            print('[AGV]: File ' + db_name + ' is already exist')
            return False
        else:
            try:
                locs_file = open(locs_fpath, 'w')
            except:
                print('[AGV]: Can\'t create file ' + db_name)
                return False
            # create corresponding header
            ctime = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            db = {"header": { "name": db_name,
                              "type": db_type,
                              "mtime": ctime,
                              "comment": "..." },
                    "points": data
                   }

            try:
                json.dump(db, locs_file, indent=2)
            except:
                print('[AGV]: Can\'t load json data into ' + db_name)
                locs_file.close()
                return False
            locs_file.close()
            return True


    # add new location in corresponding locs database
    # if locs_db already has loc_name it will be overwritten
    def add_location(self, locs_db_name, loc_name, loc_type, loc_pos, loc_or):
        locs_fpath = self.locs_dpath + locs_db_name + '.locs'
        # check if changing locs_db is not current locs_db
        if (locs_db_name != self.locs_db_name):
            if os.path.isfile(locs_fpath):
                locs_db = self.load_locs_db(locs_db_name)
            else:
                self.new_db(locs_db_name)
                locs_db = self.load_locs_db(locs_db_name)
            if locs_db is None:
                print('[AGV]: Can\'t open file ' + locs_db_name)
                return False
        else:
            locs_db = self.locs
        point = {loc_name: {"type": loc_type,
                            "pose": loc_pos,
                            "orientation": loc_or}}

        locs_db["points"].update(point)
        locs_db["header"]["mtime"] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        try:
            locs_file = open(locs_fpath, 'w')
        except:
            print('[AGV]: Can\'t open file ' + locs_db_name)
            return False

        try:
            json.dump(locs_db, locs_file, indent=2)
        except:
            print('[AGV]: Can\'t load json data into ' + locs_db_name)
            locs_file.close()
            return False
            locs_file.close()
        print('[AGV]: point ' + loc_name + ' is added to ' + locs_db_name +
              ' locations database')
        if (locs_db_name == self.locs_db_name):
            self.locs = locs_db
        return True


    # create new local locs database from remote source
    # if database with the same name is already exist
    # rewrite it and make backup
    # return False if file is already exist
    def add_remote_locs_db(self, locs_db_name, locs_db):
        if "points" in locs_db:
            locs_fpath = self.locs_dpath + locs_db_name + '.locs'
            if os.path.isfile(locs_fpath):
                locs_db_old = self.load_locs_db(locs_db_name)
                self.new_db(locs_db_name + '.bkp', "locs", locs_db_old["points"])
            return self.new_db(locs_db_name, "locs", locs_db["points"], True)
        else:
            print('[AGV]: remote db is empty')
            return False


    # return dict with all location's data
    # return False if thire is no data on it
    def get_loc_data(self, locs_db_name, loc_name):
        # check if changing locs_db is not current locs_db
        if (locs_db_name != self.locs_db_name):
            locs_fpath = self.locs_dpath + locs_db_name + '.locs'
            if os.path.isfile(locs_fpath):
                locs_db = self.load_locs_db(locs_db_name)
            else:
                print('[AGV]: There in no ' + locs_db_name + ' database')
                return None
        else:
            locs_db = self.locs

        try:
            loc_data = locs_db["points"][loc_name]
            return loc_data
        except:
            print('[AGV]: There in no ' + loc_name + ' location in ' +
                   locs_db_name + ' database')
            return None

    # return local locs database
    # return False if locs_db_name does not exist
    def get_local_locs_db(self, locs_db_name):
        # check if changing locs_db is not current locs_db
        if (locs_db_name != self.locs_db_name):
            locs_fpath = self.locs_dpath + locs_db_name + '.locs'
            if os.path.isfile(locs_fpath):
                return  self.load_locs_db(locs_db_name)
            else:
                print('[AGV]: There in no ' + locs_db_name + ' database')
                return None
        else:
            return self.locs
