#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file FingerVisionFilter.py
 @brief Filter module for FingerVision
 @date $Date$


"""
import math
import numpy as np
import numpy.linalg as la
import FingerVision
import OpenRTM_aist
import RTC
import sys
import time
sys.path.append(".")


RIGHT = 0
LEFT = 1

# Import RTM module


# Import Service implementation class
# <rtc-template block="service_impl">

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
fingervisionfilter_spec = ["implementation_id", "FingerVisionFilter",
                           "type_name",         "FingerVisionFilter",
                           "description",       "Filter module for FingerVision",
                           "version",           "1.0.0",
                           "vendor",            "MasutaniLab",
                           "category",          "Sensor",
                           "activity_type",     "STATIC",
                           "max_instance",      "1",
                           "language",          "Python",
                           "lang_type",         "SCRIPT",
                           "conf.default.fv", "fv",
                           "conf.default.side", "r",

                           "conf.__widget__.fv", "text",
                           "conf.__widget__.side", "text",

                           "conf.__type__.fv", "string",
                           "conf.__type__.side", "string",

                           ""]
# </rtc-template>

# Matlab-like mod function that returns always positive
def Mod(x, y):
  if y==0:  return x
  return x-y*math.floor(x/y)

# Container class that can hold any variables
# ref. http://blog.beanz-net.jp/happy_programming/2008/11/python-5.html

class TContainerCore(object):
    def __init__(self):
        pass

    def __del__(self):
        pass

    def __str__(self):
        return str(self.__dict__)

    def __repr__(self):
        return str(self.__dict__)

    def __iter__(self):
        return self.__dict__.__iter__()

    def items(self):
        return self.__dict__.items()

    def iteritems(self):
        return list(self.__dict__.items())

    def keys(self):
        return self.__dict__.keys()

    def values(self):
        return self.__dict__.values()

    def __getitem__(self, key):
        return self.__dict__[key]

    def __setitem__(self, key, value):
        self.__dict__[key] = value

    def __delitem__(self, key):
        del self.__dict__[key]

    def __contains__(self, key):
        return key in self.__dict__

    def Cleanup(self):
        keys = self.__dict__.keys()
        for k in keys:
            self.__dict__[k] = None
            del self.__dict__[k]


class TContainerDebug(TContainerCore):
    def __init__(self):
        super(TContainerDebug, self).__init__()
        print('Created TContainer object', hex(id(self)))

    def __del__(self):
        super(TContainerDebug, self).__del__()
        print('Deleting TContainer object', hex(id(self)))


'''Helper function to generate a container object.
  Note: the function name is like a class name;
    this is because originally TContainer was a class
    where TContainerCore and TContainerDebug are unified.'''


def TContainer(debug=False):
    return TContainerCore() if not debug else TContainerDebug()


def BlobMoves(bm, fv, side, wr):
    cx = bm.width/2
    cy = bm.height/2
    div = float(bm.width+bm.height)/2.0

    def convert_raw(mv):
        fscale = [1.0, 1.0, 1.0]
        rxy = ((mv.Pox-cx)/div, (mv.Poy-cy)/div)
        fxy = (mv.DPx, mv.DPy)
        fz = la.norm(fxy)  # max(0.0,mv.DS)
        if side == RIGHT:
            f = [+(fxy[0]*fscale[0]), -
                 (fz*fscale[2]), -(fxy[1]*fscale[1])]
            p = [+rxy[0], -rxy[1]]
        elif side == LEFT:
            f = [-(fxy[0]*fscale[0]), +
                 (fz*fscale[2]), -(fxy[1]*fscale[1])]
            p = [-rxy[0], -rxy[1]]
        else:
            raise NotImplementedError(
                'BlobMoves: side not in (0,1) is not considered.')
        return p+f

    def convert_wrench(p_f):
        p, f = p_f[:2], p_f[2:]
        tscale = 1.0
        tau = np.cross([p[0], 0.0, p[1]], f)*tscale
        return f+tau.tolist()

    def convert_dstate(p_f):
        p, f = p_f[:2], p_f[2:]
        fz = abs(f[1])
        if fz < 0.8:
            dstate = 0
        elif fz < 1.8:
            dstate = 1
        elif fz < 2.5:
            dstate = 3
        else:
            dstate = 5
        return dstate

    posforce_array = [convert_raw(mv) for mv in bm.data]
    force_array = [convert_wrench(p_f) for p_f in posforce_array]
    dstate_array = [convert_dstate(p_f) for p_f in posforce_array]
    if len(force_array) > 0:
        force = [sum([force[d] for force in force_array]) /
                 float(len(force_array)) for d in range(6)]
    else:
        force = []
    dstate = sum(dstate_array)

    wr.seq = bm.seq
    wr.tm = bm.tm
    wr.frame_id = bm.frame_id
    wr.fv = fv
    wr.posforce_array = np.array(posforce_array).ravel().tolist()  # Serialized
    wr.force_array = np.array(force_array).ravel().tolist()  # Serialized
    wr.dstate_array = dstate_array
    wr.force = force
    wr.dstate = dstate


def MovingAverageFilter(value, log, length):
    log.append(value)
    if len(log) > length:
        log.pop(0)
    return np.mean(log, 0)


def ProxVision(pv, fv, oi, options, state):
    cx = pv.width/2
    cy = pv.height/2
    div = float(pv.width+pv.height)/2.0
    diva = float(pv.width*pv.height)
    # Object and movement detection (shrunken form, e.g. 3x3 mat)
    # FIXME: Do not divide by diva since MvS elements are already normalized by the local area!!! 255 would be better
    obj_s = [i/diva for i in pv.ObjS]
    # FIXME: Do not divide by diva since MvS elements are already normalized by the local area!!! 255 would be better
    mv_s = [100.0*i/diva for i in pv.MvS]
    # Get object center and orientation from moment
    m00, m10, m01 = pv.ObjM_m[:3]
    mu20, mu11, mu02 = pv.ObjM_mu[:3]
    if m00 > 0.0:
        obj_center = [(m10/m00-cx)/div, (m01/m00-cy)/div]
        obj_orientation = 0.5*math.atan2(2.0*mu11/m00, (mu20/m00-mu02/m00))
        obj_area = m00/diva
    else:
        obj_center = [0.0, 0.0]
        obj_orientation = 0.0
        obj_area = 0.0

    # Temporal filters:
    if 'state_initialized' not in state:
        state.state_initialized = True
        state.last_tm = pv.tm
        state.last_obj_center = obj_center
        state.last_obj_orientation = obj_orientation
        state.last_obj_area = obj_area
        state.obj_area_log = []
        state.d_obj_center_log = []
        state.d_obj_orientation_log = []
        state.d_obj_area_log = []

    dt = pv.tm.sec-state.last_tm.sec + 1e-9*(pv.tm.nsec-state.last_tm.nsec)
    if dt > 0.0:
        def angle_mod(q):
            return Mod(q+0.5*math.pi, math.pi)-0.5*math.pi
        d_obj_center = [
            (state.last_obj_center[i]-obj_center[i])/dt for i in (0, 1)]
        d_obj_orientation = angle_mod(
            state.last_obj_orientation-obj_orientation)/dt
        d_obj_area = (state.last_obj_area-obj_area)/dt
    else:
        d_obj_center = [0.0, 0.0]
        d_obj_orientation = 0.0
        d_obj_area = 0.0

    state.last_tm = pv.tm
    state.last_obj_center = obj_center
    state.last_obj_orientation = obj_orientation
    state.last_obj_area = obj_area

    obj_area_filtered = MovingAverageFilter(
        obj_area, state.obj_area_log, options['filter_len'])
    d_obj_center_filtered = MovingAverageFilter(
        d_obj_center, state.d_obj_center_log, options['filter_len']).tolist()
    d_obj_orientation_filtered = MovingAverageFilter(
        d_obj_orientation, state.d_obj_orientation_log, options['filter_len'])
    d_obj_area_filtered = MovingAverageFilter(
        d_obj_area, state.d_obj_area_log, options['filter_len'])

    oi.seq = pv.seq
    oi.tm = pv.tm
    oi.frame_id = pv.frame_id
    oi.fv = fv
    oi.mv_s = mv_s
    oi.obj_s = obj_s
    oi.obj_center = obj_center
    oi.obj_orientation = obj_orientation
    oi.obj_area = obj_area
    oi.d_obj_center = d_obj_center
    oi.d_obj_orientation = d_obj_orientation
    oi.d_obj_area = d_obj_area
    oi.obj_area_filtered = obj_area_filtered
    oi.d_obj_center_filtered = d_obj_center_filtered
    oi.d_obj_orientation_filtered = d_obj_orientation_filtered
    oi.d_obj_area_filtered = d_obj_area_filtered

##
# @class FingerVisionFilter
# @brief Filter module for FingerVision
#
#


class FingerVisionFilter(OpenRTM_aist.DataFlowComponentBase):

        ##
        # @brief constructor
        # @param manager Maneger Object
        #
    def __init__(self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

        self._d_blob_moves = OpenRTM_aist.instantiateDataType(
            FingerVision.BlobMoves)
        """
        """
        self._blob_movesIn = OpenRTM_aist.InPort(
            "blob_moves", self._d_blob_moves)
        self._d_prox_vision = OpenRTM_aist.instantiateDataType(
            FingerVision.ProxVision)
        """
        """
        self._prox_visionIn = OpenRTM_aist.InPort(
            "prox_vision", self._d_prox_vision)
        self._d_fv_filter1_wrench = OpenRTM_aist.instantiateDataType(
            FingerVision.Filter1Wrench)
        """
        """
        self._fv_filter1_wrenchOut = OpenRTM_aist.OutPort(
            "fv_filter1_wrench", self._d_fv_filter1_wrench)
        self._d_fv_filter1_objinfo = OpenRTM_aist.instantiateDataType(
            FingerVision.Filter1ObjInfo)
        """
        """
        self._fv_filter1_objinfoOut = OpenRTM_aist.OutPort(
            "fv_filter1_objinfo", self._d_fv_filter1_objinfo)

        # ロガー
        self.log = OpenRTM_aist.Manager.instance().getLogbuf("FingerVisionFilter")

        # initialize of configuration-data.
        # <rtc-template block="init_conf_param">
        """
        
         - Name:  fv
         - DefaultValue: fv
        """
        self._fv = ['fv']
        """
        
         - Name:  side
         - DefaultValue: r
        """
        self._side = ['r']

        # </rtc-template>

    ##
    #
    # The initialize action (on CREATED->ALIVE transition)
    # formaer rtc_init_entry()
    #
    # @return RTC::ReturnCode_t
    #
    #

    def onInitialize(self):
        self.log.RTC_INFO("onInitialize()")
        # Bind variables and configuration variable
        self.bindParameter("fv", self._fv, "fv")
        self.bindParameter("side", self._side, "r")

        # Set InPort buffers
        self.addInPort("blob_moves", self._blob_movesIn)
        self.addInPort("prox_vision", self._prox_visionIn)

        # Set OutPort buffers
        self.addOutPort("fv_filter1_wrench", self._fv_filter1_wrenchOut)
        self.addOutPort("fv_filter1_objinfo", self._fv_filter1_objinfoOut)

        # Set service provider to Ports

        # Set service consumers to Ports

        # Set CORBA Service Ports

        return RTC.RTC_OK

    ###
    ##
    # The finalize action (on ALIVE->END transition)
    # formaer rtc_exiting_entry()
    ##
    # @return RTC::ReturnCode_t
    #
    ##
    # def onFinalize(self):
    #
    #	return RTC.RTC_OK

    ###
    ##
    # The startup action when ExecutionContext startup
    # former rtc_starting_entry()
    ##
    # @param ec_id target ExecutionContext Id
    ##
    # @return RTC::ReturnCode_t
    ##
    ##
    # def onStartup(self, ec_id):
    #
    #	return RTC.RTC_OK

    ###
    ##
    # The shutdown action when ExecutionContext stop
    # former rtc_stopping_entry()
    ##
    # @param ec_id target ExecutionContext Id
    ##
    # @return RTC::ReturnCode_t
    ##
    ##
    # def onShutdown(self, ec_id):
    #
    #	return RTC.RTC_OK

    ##
    #
    # The activated action (Active state entry action)
    # former rtc_active_entry()
    #
    # @param ec_id target ExecutionContext Id
    #
    # @return RTC::ReturnCode_t
    #
    #
    def onActivated(self, ec_id):
        self.log.RTC_INFO("onActivated()")
        self.fv = self._fv[0]
        if self._side[0] == 'r':
            self.side = 0
        elif self._side[0] == 'l':
            self.side = 1
        else:
            self.side = -1

        self.options_fobjinfo = {
            'filter_len': 5,
        }

        self.state_fobjinfo = TContainer(debug=True)

        return RTC.RTC_OK

    ##
    #
    # The deactivated action (Active state exit action)
    # former rtc_active_exit()
    #
    # @param ec_id target ExecutionContext Id
    #
    # @return RTC::ReturnCode_t
    #
    #
    def onDeactivated(self, ec_id):
        self.log.RTC_INFO("onDeactivated()")

        return RTC.RTC_OK

    ##
    #
    # The execution action that is invoked periodically
    # former rtc_active_do()
    #
    # @param ec_id target ExecutionContext Id
    #
    # @return RTC::ReturnCode_t
    #
    #

    def onExecute(self, ec_id):
        if self._blob_movesIn.isNew():
            bm = self._blob_movesIn.read()
            # print('bm: ', bm)
            wr = self._d_fv_filter1_wrench
            BlobMoves(bm, self.fv, self.side, wr)
            # print('wr: ', wr)
            self._fv_filter1_wrenchOut.write()

        if self._prox_visionIn.isNew():
            pv = self._prox_visionIn.read()
            # print('pv: ', pv)
            oi = self._d_fv_filter1_objinfo
            ProxVision(pv, self.fv, oi,
                       self.options_fobjinfo,
                       self.state_fobjinfo)
            # print('oi: ', oi)
            self._fv_filter1_objinfoOut.write()

        return RTC.RTC_OK

        ###
        ##
        # The aborting action when main logic error occurred.
        # former rtc_aborting_entry()
        ##
        # @param ec_id target ExecutionContext Id
        ##
        # @return RTC::ReturnCode_t
        ##
        ##
        # def onAborting(self, ec_id):
        #
        #	return RTC.RTC_OK

        ###
        ##
        # The error action in ERROR state
        # former rtc_error_do()
        ##
        # @param ec_id target ExecutionContext Id
        ##
        # @return RTC::ReturnCode_t
        ##
        ##
        # def onError(self, ec_id):
        #
        #	return RTC.RTC_OK

        ###
        ##
        # The reset action that is invoked resetting
        # This is same but different the former rtc_init_entry()
        ##
        # @param ec_id target ExecutionContext Id
        ##
        # @return RTC::ReturnCode_t
        ##
        ##
        # def onReset(self, ec_id):
        #
        #	return RTC.RTC_OK

        ###
        ##
        # The state update action that is invoked after onExecute() action
        # no corresponding operation exists in OpenRTm-aist-0.2.0
        ##
        # @param ec_id target ExecutionContext Id
        ##
        # @return RTC::ReturnCode_t
        ##

        ##
        # def onStateUpdate(self, ec_id):
        #
        #	return RTC.RTC_OK

        ###
        ##
        # The action that is invoked when execution context's rate is changed
        # no corresponding operation exists in OpenRTm-aist-0.2.0
        ##
        # @param ec_id target ExecutionContext Id
        ##
        # @return RTC::ReturnCode_t
        ##
        ##
        # def onRateChanged(self, ec_id):
        #
        #	return RTC.RTC_OK


def FingerVisionFilterInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=fingervisionfilter_spec)
    manager.registerFactory(profile,
                            FingerVisionFilter,
                            OpenRTM_aist.Delete)


def MyModuleInit(manager):
    FingerVisionFilterInit(manager)

    # Create a component
    comp = manager.createComponent("FingerVisionFilter")


def main():
    mgr = OpenRTM_aist.Manager.init(sys.argv)
    mgr.setModuleInitProc(MyModuleInit)
    mgr.activateManager()
    mgr.runManager()


if __name__ == "__main__":
    main()
