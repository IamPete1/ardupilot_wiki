
AC_AttitudeControl
==================

 desc

.. code-block::

   table

----

AC_AttitudeControl.get_rpy_srate
================================

 return slew rates for VTOL controller

@\ *return* — roll slew rate

@\ *return* — pitch slew rate

@\ *return* — yaw slew rate

.. code-block::

   (method) AC_AttitudeControl:get_rpy_srate()
     -> number
     2. number
     3. number

----

AP_Camera__camera_state_t
=========================

.. code-block::

   function AP_Camera__camera_state_t()
     -> AP_Camera__camera_state_t_ud

----

AP_Camera__camera_state_t_ud
============================

 desc

focus_type
----------

.. code-block::

   (method) AP_Camera__camera_state_t_ud:focus_type()
     -> integer

 get field

focus_value
-----------

.. code-block::

   (method) AP_Camera__camera_state_t_ud:focus_value()
     -> number

 get field

recording_video
---------------

.. code-block::

   (method) AP_Camera__camera_state_t_ud:recording_video()
     -> boolean

 get field

take_pic_incr
-------------

.. code-block::

   (method) AP_Camera__camera_state_t_ud:take_pic_incr()
     -> integer

 get field

tracking_p1
-----------

.. code-block::

   (method) AP_Camera__camera_state_t_ud:tracking_p1()
     -> Vector2f_ud

 get field

tracking_p2
-----------

.. code-block::

   (method) AP_Camera__camera_state_t_ud:tracking_p2()
     -> Vector2f_ud

 get field

tracking_type
-------------

.. code-block::

   (method) AP_Camera__camera_state_t_ud:tracking_type()
     -> integer

 get field

zoom_type
---------

.. code-block::

   (method) AP_Camera__camera_state_t_ud:zoom_type()
     -> integer

 get field

zoom_value
----------

.. code-block::

   (method) AP_Camera__camera_state_t_ud:zoom_value()
     -> number

 get field

----

AP_EFI_Backend_ud
=================

 desc

handle_scripting
----------------

.. code-block::

   (method) AP_EFI_Backend_ud:handle_scripting(state: EFI_State_ud)
     -> boolean

 desc

----

AP_HAL__AnalogSource_ud
=======================

 desc

set_pin
-------

.. code-block::

   (method) AP_HAL__AnalogSource_ud:set_pin(pin_number: integer)
     -> boolean

 desc

voltage_average
---------------

.. code-block::

   (method) AP_HAL__AnalogSource_ud:voltage_average()
     -> number

 desc

voltage_average_ratiometric
---------------------------

.. code-block::

   (method) AP_HAL__AnalogSource_ud:voltage_average_ratiometric()
     -> number

 desc

voltage_latest
--------------

.. code-block::

   (method) AP_HAL__AnalogSource_ud:voltage_latest()
     -> number

 desc

----

AP_HAL__I2CDevice_ud
====================

 desc

read_registers
--------------

.. code-block::

   (method) AP_HAL__I2CDevice_ud:read_registers(register_num: integer, read_length?: integer)
     -> integer|table|nil

 If no read length is provided a single register will be read and returned.
 If read length is provided a table of register values are returned.

set_address
-----------

.. code-block::

   (method) AP_HAL__I2CDevice_ud:set_address(address: integer)

 desc

set_retries
-----------

.. code-block::

   (method) AP_HAL__I2CDevice_ud:set_retries(retries: integer)

 desc

write_register
--------------

.. code-block::

   (method) AP_HAL__I2CDevice_ud:write_register(register_num: integer, value: integer)
     -> boolean

 desc

----

AP_HAL__PWMSource_ud
====================

 desc

get_pwm_avg_us
--------------

.. code-block::

   (method) AP_HAL__PWMSource_ud:get_pwm_avg_us()
     -> integer

 desc

get_pwm_us
----------

.. code-block::

   (method) AP_HAL__PWMSource_ud:get_pwm_us()
     -> integer

 desc

set_pin
-------

.. code-block::

   (method) AP_HAL__PWMSource_ud:set_pin(pin_number: integer)
     -> boolean

 desc

----

AP_HAL__UARTDriver_ud
=====================

 Serial driver object

available
---------

.. code-block::

   (method) AP_HAL__UARTDriver_ud:available()
     -> uint32_t_ud

 Returns number of available bytes to read.

begin
-----

.. code-block::

   (method) AP_HAL__UARTDriver_ud:begin(baud_rate: number|uint32_t_ud)

 Start serial port with given baud rate

read
----

.. code-block::

   (method) AP_HAL__UARTDriver_ud:read()
     -> integer

 Read a single byte from the serial port

@\ *return* — byte, -1 if not available

readstring
----------

.. code-block::

   (method) AP_HAL__UARTDriver_ud:readstring(count: integer)
     -> string|nil

read count bytes from a uart and return as a lua string. Note

  that the returned string can be shorter than the requested length
-------------------------------------------------------------------

set_flow_control
----------------

.. code-block::

   (method) AP_HAL__UARTDriver_ud:set_flow_control(flow_control_setting: integer|'0'|'1'|'2')

 Set flow control option for serial port

.. code-block::

   flow_control_setting:
       | '0' -- disabled
       | '1' -- enabled
       | '2' -- auto

write
-----

.. code-block::

   (method) AP_HAL__UARTDriver_ud:write(value: integer)
     -> uint32_t_ud

 Writes a single byte

@\ *param* ``value`` — byte to write

@\ *return* — 1 if success else 0

----

AP_Proximity_Backend_ud
=======================

 Proximity backend methods

handle_script_3d_msg
--------------------

.. code-block::

   (method) AP_Proximity_Backend_ud:handle_script_3d_msg(vector_3d: Vector3f_ud, update_boundary: boolean)
     -> boolean

 send 3d object as 3d vector

handle_script_distance_msg
--------------------------

.. code-block::

   (method) AP_Proximity_Backend_ud:handle_script_distance_msg(dist_m: number, yaw_deg: number, pitch_deg: number, update_boundary: boolean)
     -> boolean

 send 3d object as angles

set_distance_min_max
--------------------

.. code-block::

   (method) AP_Proximity_Backend_ud:set_distance_min_max(min: number, max: number)
     -> boolean

 Set sensor min and max. Only need to do it once

type
----

.. code-block::

   (method) AP_Proximity_Backend_ud:type()
     -> integer

 type of backend

update_virtual_boundary
-----------------------

.. code-block::

   (method) AP_Proximity_Backend_ud:update_virtual_boundary()
     -> boolean

 Push virtual proximity boundary into actual boundary

----

AP_RangeFinder_Backend_ud
=========================

 RangeFinder backend

distance
--------

.. code-block::

   (method) AP_RangeFinder_Backend_ud:distance()
     -> number

 Current distance of the sensor instance

get_state
---------

.. code-block::

   (method) AP_RangeFinder_Backend_ud:get_state()
     -> RangeFinder_State_ud

 State of most recent range finder measurment

handle_script_msg
-----------------

.. code-block::

   (method) AP_RangeFinder_Backend_ud:handle_script_msg(state: number|RangeFinder_State_ud)
     -> boolean

 Send range finder measurement to lua rangefinder backend. Returns false if failed

orientation
-----------

.. code-block::

   (method) AP_RangeFinder_Backend_ud:orientation()
     -> integer

 Orintation of the rangefinder of this instance

signal_quality
--------------

.. code-block::

   (method) AP_RangeFinder_Backend_ud:signal_quality()
     -> number

 Current distance measurement signal_quality of the sensor instance

status
------

.. code-block::

   (method) AP_RangeFinder_Backend_ud:status()
     -> integer

 Status of this rangefinder instance

type
----

.. code-block::

   (method) AP_RangeFinder_Backend_ud:type()
     -> integer

 Type of rangefinder of this instance

----

AR_AttitudeControl
==================

 desc

.. code-block::

   table

----

AR_AttitudeControl.get_srate
============================

 return attitude controller slew rates for rovers

@\ *return* — steering slew rate

@\ *return* — spees slew rate

.. code-block::

   (method) AR_AttitudeControl:get_srate()
     -> number
     2. number

----

AR_PosControl
=============

 desc

.. code-block::

   table

----

AR_PosControl.get_srate
=======================

 return position controller slew rates for rovers

@\ *return* — velocity slew rate

.. code-block::

   (method) AR_PosControl:get_srate()
     -> number

----

BattMonitorScript_State
=======================

.. code-block::

   function BattMonitorScript_State()
     -> BattMonitorScript_State_ud

----

BattMonitorScript_State_ud
==========================

 desc

capacity_remaining_pct
----------------------

.. code-block::

   (method) BattMonitorScript_State_ud:capacity_remaining_pct(value: integer)

 set field

cell_count
----------

.. code-block::

   (method) BattMonitorScript_State_ud:cell_count(value: integer)

 set field

cell_voltages
-------------

.. code-block::

   (method) BattMonitorScript_State_ud:cell_voltages(index: integer, value: integer)

 set array field

consumed_mah
------------

.. code-block::

   (method) BattMonitorScript_State_ud:consumed_mah(value: number)

 set field

consumed_wh
-----------

.. code-block::

   (method) BattMonitorScript_State_ud:consumed_wh(value: number)

 set field

current_amps
------------

.. code-block::

   (method) BattMonitorScript_State_ud:current_amps(value: number)

 set field

cycle_count
-----------

.. code-block::

   (method) BattMonitorScript_State_ud:cycle_count(value: integer)

 set field

healthy
-------

.. code-block::

   (method) BattMonitorScript_State_ud:healthy(value: boolean)

 set field

temperature
-----------

.. code-block::

   (method) BattMonitorScript_State_ud:temperature(value: number)

 set field

voltage
-------

.. code-block::

   (method) BattMonitorScript_State_ud:voltage(value: number)

 set field

----

CAN
===

 CAN bus interaction

.. code-block::

   table

----

CAN.get_device
==============

 get a CAN bus device handler first scripting driver, will return nil if no driver with protocol Scripting is configured

@\ *param* ``buffer_len`` — buffer length 1 to 25

.. code-block::

   (method) CAN:get_device(buffer_len: number|uint32_t_ud)
     -> ScriptingCANBuffer_ud|nil

----

CAN.get_device2
===============

 get a CAN bus device handler second scripting driver, will return nil if no driver with protocol Scripting2 is configured

@\ *param* ``buffer_len`` — buffer length 1 to 25

.. code-block::

   (method) CAN:get_device2(buffer_len: number|uint32_t_ud)
     -> ScriptingCANBuffer_ud|nil

----

CANFrame
========

.. code-block::

   function CANFrame()
     -> CANFrame_ud

----

CANFrame_ud
===========

 desc

data
----

.. code-block::

   (method) CANFrame_ud:data(index: integer)
     -> integer

 get array field

dlc
---

.. code-block::

   (method) CANFrame_ud:dlc()
     -> integer

 get field

id
--

.. code-block::

   (method) CANFrame_ud:id()
     -> uint32_t_ud

 get field

id_signed
---------

.. code-block::

   (method) CANFrame_ud:id_signed()
     -> integer

 desc

isErrorFrame
------------

.. code-block::

   (method) CANFrame_ud:isErrorFrame()
     -> boolean

 desc

isExtended
----------

.. code-block::

   (method) CANFrame_ud:isExtended()
     -> boolean

 desc

isRemoteTransmissionRequest
---------------------------

.. code-block::

   (method) CANFrame_ud:isRemoteTransmissionRequest()
     -> boolean

 desc

----

Cylinder_Status
===============

.. code-block::

   function Cylinder_Status()
     -> Cylinder_Status_ud

----

Cylinder_Status_ud
==================

 EFI Cylinder_Status structure

cylinder_head_temperature
-------------------------

.. code-block::

   (method) Cylinder_Status_ud:cylinder_head_temperature()
     -> number

 get field

cylinder_head_temperature2
--------------------------

.. code-block::

   (method) Cylinder_Status_ud:cylinder_head_temperature2()
     -> number

 get field

exhaust_gas_temperature
-----------------------

.. code-block::

   (method) Cylinder_Status_ud:exhaust_gas_temperature()
     -> number

 get field

exhaust_gas_temperature2
------------------------

.. code-block::

   (method) Cylinder_Status_ud:exhaust_gas_temperature2()
     -> number

 get field

ignition_timing_deg
-------------------

.. code-block::

   (method) Cylinder_Status_ud:ignition_timing_deg()
     -> number

 get field

injection_time_ms
-----------------

.. code-block::

   (method) Cylinder_Status_ud:injection_time_ms()
     -> number

 get field

lambda_coefficient
------------------

.. code-block::

   (method) Cylinder_Status_ud:lambda_coefficient()
     -> number

 get field

----

EFI_State
=========

.. code-block::

   function EFI_State()
     -> EFI_State_ud

----

EFI_State_ud
============

 EFI state structure

atmospheric_pressure_kpa
------------------------

.. code-block::

   (method) EFI_State_ud:atmospheric_pressure_kpa()
     -> number

 get field

coolant_temperature
-------------------

.. code-block::

   (method) EFI_State_ud:coolant_temperature()
     -> number

 get field

cylinder_status
---------------

.. code-block::

   (method) EFI_State_ud:cylinder_status()
     -> Cylinder_Status_ud

 get field

ecu_index
---------

.. code-block::

   (method) EFI_State_ud:ecu_index()
     -> integer

 get field

engine_load_percent
-------------------

.. code-block::

   (method) EFI_State_ud:engine_load_percent()
     -> integer

 get field

engine_speed_rpm
----------------

.. code-block::

   (method) EFI_State_ud:engine_speed_rpm()
     -> uint32_t_ud

 get field

estimated_consumed_fuel_volume_cm3
----------------------------------

.. code-block::

   (method) EFI_State_ud:estimated_consumed_fuel_volume_cm3()
     -> number

 get field

fuel_consumption_rate_cm3pm
---------------------------

.. code-block::

   (method) EFI_State_ud:fuel_consumption_rate_cm3pm()
     -> number

 get field

fuel_pressure
-------------

.. code-block::

   (method) EFI_State_ud:fuel_pressure()
     -> number

 get field

fuel_pressure_status
--------------------

.. code-block::

   (method) EFI_State_ud:fuel_pressure_status()
     -> integer|'0'|'1'|'2'|'3'

 get field

.. code-block::

   return #1:
       | '0' -- Not supported
       | '1' -- Ok
       | '2' -- Below nominal
       | '3' -- Above nominal

general_error
-------------

.. code-block::

   (method) EFI_State_ud:general_error()
     -> boolean

 get field

ignition_voltage
----------------

.. code-block::

   (method) EFI_State_ud:ignition_voltage()
     -> number

 get field

intake_manifold_pressure_kpa
----------------------------

.. code-block::

   (method) EFI_State_ud:intake_manifold_pressure_kpa()
     -> number

 get field

intake_manifold_temperature
---------------------------

.. code-block::

   (method) EFI_State_ud:intake_manifold_temperature()
     -> number

 get field

last_updated_ms
---------------

.. code-block::

   (method) EFI_State_ud:last_updated_ms()
     -> uint32_t_ud

 get field

oil_pressure
------------

.. code-block::

   (method) EFI_State_ud:oil_pressure()
     -> number

 get field

oil_temperature
---------------

.. code-block::

   (method) EFI_State_ud:oil_temperature()
     -> number

 get field

pt_compensation
---------------

.. code-block::

   (method) EFI_State_ud:pt_compensation()
     -> number

 get field

spark_dwell_time_ms
-------------------

.. code-block::

   (method) EFI_State_ud:spark_dwell_time_ms()
     -> number

 get field

throttle_out
------------

.. code-block::

   (method) EFI_State_ud:throttle_out()
     -> number

 get field

throttle_position_percent
-------------------------

.. code-block::

   (method) EFI_State_ud:throttle_position_percent()
     -> integer

 get field

----

ESCTelemetryData
================

.. code-block::

   function ESCTelemetryData()
     -> ESCTelemetryData_ud

----

ESCTelemetryData_ud
===================

 desc

consumption_mah
---------------

.. code-block::

   (method) ESCTelemetryData_ud:consumption_mah(value: number)

 set consumption

current
-------

.. code-block::

   (method) ESCTelemetryData_ud:current(value: number)

 set current

motor_temp_cdeg
---------------

.. code-block::

   (method) ESCTelemetryData_ud:motor_temp_cdeg(value: integer)

 set motor temperature

temperature_cdeg
----------------

.. code-block::

   (method) ESCTelemetryData_ud:temperature_cdeg(value: integer)

 set temperature

voltage
-------

.. code-block::

   (method) ESCTelemetryData_ud:voltage(value: number)

 set voltage

----

FWVersion
=========

 desc

.. code-block::

   table

----

FWVersion.hash
==============

 get field

.. code-block::

   (method) FWVersion:hash()
     -> string

----

FWVersion.major
===============

 get field

.. code-block::

   (method) FWVersion:major()
     -> integer

----

FWVersion.minor
===============

 get field

.. code-block::

   (method) FWVersion:minor()
     -> integer

----

FWVersion.patch
===============

 get field

.. code-block::

   (method) FWVersion:patch()
     -> integer

----

FWVersion.string
================

 get field

.. code-block::

   (method) FWVersion:string()
     -> string

----

FWVersion.type
==============

get APM\ *BUILD*\ ? value from AP_Vehicle/AP_Vehicle_Type.h that is checked against APM_BUILD_TYPE()

.. code-block::

   return #1:
       | '1' -- Rover
       | '2' -- ArduCopter
       | '3' -- ArduPlane
       | '4' -- AntennaTracker
       | '7' -- ArduSub
       | '9' -- AP_Periph
       | '12' -- Blimp
       | '13' -- Heli

.. code-block::

   (method) FWVersion:type()
     -> integer|'1'|'12'|'13'|'2'...(+4)

----

LED
===

 desc

.. code-block::

   table

----

LED.get_rgb
===========

 desc

.. code-block::

   (method) LED:get_rgb()
     -> integer
     2. integer
     3. integer

----

Location
========

 Create location object

.. code-block::

   function Location()
     -> Location_ud

----

Location_ud
===========

 Location is a userdata object that holds locations expressed as latitude, longitude, altitude.
 The altitude can be in several different frames, relative to home, absolute altitude above mean sea level, or relative to terrain.
 To create a new Location userdata you can call Location() to allocate an empty location object, or call a method that returns one to you.

alt
---

.. code-block::

   (method) Location_ud:alt()
     -> integer

 get altitude in cm

@\ *return* — altitude in cm

change_alt_frame
----------------

.. code-block::

   (method) Location_ud:change_alt_frame(desired_frame: integer|'0'|'1'|'2'|'3')
     -> boolean

 Set the altitude frame of this location

@\ *param* ``desired_frame`` — altitude frame

.. code-block::

   desired_frame:
       | '0' -- ABSOLUTE
       | '1' -- ABOVE_HOME
       | '2' -- ABOVE_ORIGIN
       | '3' -- ABOVE_TERRAIN

copy
----

.. code-block::

   (method) Location_ud:copy()
     -> Location_ud

 Copy this location returning a new userdata object

@\ *return* — a copy of this location

get_alt_frame
-------------

.. code-block::

   (method) Location_ud:get_alt_frame()
     -> integer|'0'|'1'|'2'|'3'

 get altitude frame of this location

.. code-block::

   return #1:
       | '0' -- ABSOLUTE
       | '1' -- ABOVE_HOME
       | '2' -- ABOVE_ORIGIN
       | '3' -- ABOVE_TERRAIN

get_bearing
-----------

.. code-block::

   (method) Location_ud:get_bearing(loc: Location_ud)
     -> number

 Given a Location this calculates the relative bearing to the location in radians

@\ *param* ``loc`` — location to compare with

@\ *return* — bearing in radians

get_distance
------------

.. code-block::

   (method) Location_ud:get_distance(loc: Location_ud)
     -> number

 Given a Location this calculates the horizontal distance between the two locations in meters.

@\ *param* ``loc`` — location to compare with

@\ *return* — horizontal distance in meters

get_distance_NE
---------------

.. code-block::

   (method) Location_ud:get_distance_NE(loc: Location_ud)
     -> Vector2f_ud

 Given a Location this calculates the north and east distance between the two locations in meters.

@\ *param* ``loc`` — location to compare with

@\ *return* — North east distance vector in meters

get_distance_NED
----------------

.. code-block::

   (method) Location_ud:get_distance_NED(loc: Location_ud)
     -> Vector3f_ud

 Given a Location this calculates the north, east and down distance between the two locations in meters.

@\ *param* ``loc`` — location to compare with

@\ *return* — North east down distance vector in meters

get_vector_from_origin_NEU
--------------------------

.. code-block::

   (method) Location_ud:get_vector_from_origin_NEU()
     -> Vector3f_ud|nil

 Returns the offset from the EKF origin to this location.
 Returns nil if the EKF origin wasn’t available at the time this was called.

@\ *return* — Vector between origin and location north east up in meters

lat
---

.. code-block::

   (method) Location_ud:lat()
     -> integer

 get latitude in degrees * 1e7

@\ *return* — latitude in degrees * 1e7

lng
---

.. code-block::

   (method) Location_ud:lng()
     -> integer

 get longitude in degrees * 1e7

@\ *return* — longitude in degrees * 1e7

loiter_xtrack
-------------

.. code-block::

   (method) Location_ud:loiter_xtrack()
     -> boolean

 get loiter xtrack

@\ *return* — Get if the location is used for a loiter location this flags if the aircraft should track from the center point, or from the exit location of the loiter.

offset
------

.. code-block::

   (method) Location_ud:offset(ofs_north: number, ofs_east: number)

 Translates this Location by the specified north and east distance in meters.

@\ *param* ``ofs_north`` — north offset in meters

@\ *param* ``ofs_east`` — east offset in meters

offset_bearing
--------------

.. code-block::

   (method) Location_ud:offset_bearing(bearing_deg: number, distance: number)

 Translates this Location by the specified  distance given a bearing.

@\ *param* ``bearing_deg`` — bearing in degrees

@\ *param* ``distance`` — distance in meters

offset_bearing_and_pitch
------------------------

.. code-block::

   (method) Location_ud:offset_bearing_and_pitch(bearing_deg: number, pitch_deg: number, distance: number)

 Translates this Location by the specified distance given a bearing and pitch.

@\ *param* ``bearing_deg`` — bearing in degrees

@\ *param* ``pitch_deg`` — pitch in degrees

@\ *param* ``distance`` — distance in meters

origin_alt
----------

.. code-block::

   (method) Location_ud:origin_alt()
     -> boolean

 get origin alt

@\ *return* — true if altitude is relative to origin

relative_alt
------------

.. code-block::

   (method) Location_ud:relative_alt()
     -> boolean

 get relative alt

@\ *return* — true if altitude is relative to home

terrain_alt
-----------

.. code-block::

   (method) Location_ud:terrain_alt()
     -> boolean

 get terrain alt

@\ *return* — true if altitude is relative to terrain

----

LuaLS
=====

----

MotorsMatrix
============

 desc

.. code-block::

   table

----

MotorsMatrix.add_motor_raw
==========================

 desc

.. code-block::

   (method) MotorsMatrix:add_motor_raw(motor_num: integer, roll_factor: number, pitch_factor: number, yaw_factor: number, testing_order: integer)

----

MotorsMatrix.get_lost_motor
===========================

 desc get index (starting at 0) of lost motor

.. code-block::

   (method) MotorsMatrix:get_lost_motor()
     -> integer

----

MotorsMatrix.get_thrust_boost
=============================

 desc return true if we are in thrust boost due to possible lost motor

.. code-block::

   (method) MotorsMatrix:get_thrust_boost()
     -> boolean

----

MotorsMatrix.init
=================

 desc

.. code-block::

   (method) MotorsMatrix:init(expected_num_motors: integer)
     -> boolean

----

MotorsMatrix.set_throttle_factor
================================

 desc

.. code-block::

   (method) MotorsMatrix:set_throttle_factor(motor_num: integer, throttle_factor: number)
     -> boolean

----

Motors_6DoF
===========

 desc

.. code-block::

   table

----

Motors_6DoF.add_motor
=====================

 desc

.. code-block::

   (method) Motors_6DoF:add_motor(motor_num: integer, roll_factor: number, pitch_factor: number, yaw_factor: number, throttle_factor: number, forward_factor: number, right_factor: number, reversible: boolean, testing_order: integer)

----

Motors_6DoF.init
================

 desc

.. code-block::

   (method) Motors_6DoF:init(expected_num_motors: integer)
     -> boolean

----

Motors_dynamic
==============

 desc

.. code-block::

   table

----

Motors_dynamic.add_motor
========================

 desc

.. code-block::

   (method) Motors_dynamic:add_motor(motor_num: integer, testing_order: integer)

----

Motors_dynamic.init
===================

 desc

.. code-block::

   (method) Motors_dynamic:init(expected_num_motors: integer)
     -> boolean

----

Motors_dynamic.load_factors
===========================

 desc

.. code-block::

   (method) Motors_dynamic:load_factors(factor_table: motor_factor_table_ud)

----

PWMSource
=========

.. code-block::

   function PWMSource()
     -> AP_HAL__PWMSource_ud

----

Parameter
=========

 Create a new parameter helper, init must be called with a parameter name.

.. code-block::

   function Parameter()
     -> Parameter_ud

.. code-block::

   function Parameter(name: string)
     -> Parameter_ud_const

----

Parameter_ud
============

 Parameter access helper.

configured
----------

.. code-block::

   (method) Parameter_ud:configured()
     -> boolean

 Return true if the parameter has been configured by the user.

get
---

.. code-block::

   (method) Parameter_ud:get()
     -> number|nil

 Get the current value of a parameter.
 Returns nil if the init has not been called and a valid parameter found.

init
----

.. code-block::

   (method) Parameter_ud:init(name: string)
     -> boolean

 Init this parameter from a name.

init_by_info
------------

.. code-block::

   (method) Parameter_ud:init_by_info(key: integer, group_element: number|uint32_t_ud, type: integer|'1'|'2'|'3'|'4')
     -> boolean

 Init the paramter from a key. This allows the script to load old parameter that have been removed from the main code.

.. code-block::

   type:
       | '1' -- AP_PARAM_INT8
       | '2' -- AP_PARAM_INT16
       | '3' -- AP_PARAM_INT32
       | '4' -- AP_PARAM_FLOAT

set
---

.. code-block::

   (method) Parameter_ud:set(value: number)
     -> boolean

 Set the parameter to the given value. The value will not persist a reboot.

set_and_save
------------

.. code-block::

   (method) Parameter_ud:set_and_save(value: number)
     -> boolean

 Set the parameter to the given value and save. The value will be persistant after a reboot.

set_default
-----------

.. code-block::

   (method) Parameter_ud:set_default(value: number)
     -> boolean

 Set the defualt value of this parameter, if the parameter has not been configured by the user its value will be updated to the new defualt.

----

Parameter_ud_const
==================

 Parameter access helper

configured
----------

.. code-block::

   (method) Parameter_ud_const:configured()
     -> boolean

 Retrun true if the parameter has been configured by the user.

get
---

.. code-block::

   (method) Parameter_ud_const:get()
     -> number

 Get the current value of a parameter.

set
---

.. code-block::

   (method) Parameter_ud_const:set(value: number)
     -> boolean

 Set the parameter to the given value. The value will not persist a reboot.

set_and_save
------------

.. code-block::

   (method) Parameter_ud_const:set_and_save(value: number)
     -> boolean

 Set the parameter to the given value and save. The value will be persistant after a reboot.

set_default
-----------

.. code-block::

   (method) Parameter_ud_const:set_default(value: number)
     -> boolean

 Set the defualt value of this parameter, if the parameter has not been configured by the user its value will be updated to the new defualt.

----

Quaternion
==========

.. code-block::

   function Quaternion()
     -> Quaternion_ud

----

Quaternion_ud
=============

 desc

earth_to_body
-------------

.. code-block::

   (method) Quaternion_ud:earth_to_body(vec: Vector3f_ud)

 Applies rotation to vector argument

from_angular_velocity
---------------------

.. code-block::

   (method) Quaternion_ud:from_angular_velocity(angular_velocity: Vector3f_ud, time_delta: number)

 Integrates angular velocity over small time delta

from_axis_angle
---------------

.. code-block::

   (method) Quaternion_ud:from_axis_angle(axis: Vector3f_ud, angle: number)

 Constructs Quaternion from axis and angle

from_euler
----------

.. code-block::

   (method) Quaternion_ud:from_euler(roll: number, pitch: number, yaw: number)

 Construct quaternion from Euler angles

get_euler_pitch
---------------

.. code-block::

   (method) Quaternion_ud:get_euler_pitch()
     -> number

 Returns pitch component of quaternion

get_euler_roll
--------------

.. code-block::

   (method) Quaternion_ud:get_euler_roll()
     -> number

 Returns roll component of quaternion

get_euler_yaw
-------------

.. code-block::

   (method) Quaternion_ud:get_euler_yaw()
     -> number

 Returns yaw component of quaternion

inverse
-------

.. code-block::

   (method) Quaternion_ud:inverse()
     -> Quaternion_ud

 Returns inverse of quaternion

length
------

.. code-block::

   (method) Quaternion_ud:length()
     -> number

 Returns length or norm of quaternion

normalize
---------

.. code-block::

   (method) Quaternion_ud:normalize()

 Mutates quaternion have length 1

q1
--

.. code-block::

   (method) Quaternion_ud:q1()
     -> number

 get field

q2
--

.. code-block::

   (method) Quaternion_ud:q2()
     -> number

 get field

q3
--

.. code-block::

   (method) Quaternion_ud:q3()
     -> number

 get field

q4
--

.. code-block::

   (method) Quaternion_ud:q4()
     -> number

 get field

to_axis_angle
-------------

.. code-block::

   (method) Quaternion_ud:to_axis_angle(axis_angle: Vector3f_ud)

 Converts Quaternion to axis-angle representation

----

RC_Channel_ud
=============

 desc

get_aux_switch_pos
------------------

.. code-block::

   (method) RC_Channel_ud:get_aux_switch_pos()
     -> integer

 desc

norm_input
----------

.. code-block::

   (method) RC_Channel_ud:norm_input()
     -> number

 desc return input on a channel from -1 to 1, centered on the trim. Ignores the deadzone

norm_input_dz
-------------

.. code-block::

   (method) RC_Channel_ud:norm_input_dz()
     -> number

 desc return input on a channel from -1 to 1, centered on the trim. Returns zero when within deadzone of the trim

norm_input_ignore_trim
----------------------

.. code-block::

   (method) RC_Channel_ud:norm_input_ignore_trim()
     -> number

 desc

set_override
------------

.. code-block::

   (method) RC_Channel_ud:set_override(PWM: integer)

 desc

----

RPM
===

 RPM handling

.. code-block::

   table

----

RPM.get_rpm
===========

  Returns RPM of given instance, or nil if not available

@\ *param* ``instance`` — RPM instance

@\ *return* — RPM value if available

.. code-block::

   (method) RPM:get_rpm(instance: integer)
     -> number|nil

----

RangeFinder_State
=================

.. code-block::

   function RangeFinder_State()
     -> RangeFinder_State_ud

----

RangeFinder_State_ud
====================

 RangeFinder state structure

distance
--------

.. code-block::

   (method) RangeFinder_State_ud:distance()
     -> number

 get distance in meters

last_reading
------------

.. code-block::

   (method) RangeFinder_State_ud:last_reading()
     -> uint32_t_ud

 get system time (ms) of last successful update from sensor

range_valid_count
-----------------

.. code-block::

   (method) RangeFinder_State_ud:range_valid_count()
     -> integer

 get number of consecutive valid readings (max out at 10)

signal_quality
--------------

.. code-block::

   (method) RangeFinder_State_ud:signal_quality()
     -> integer

 get measurement quality in percent 0-100, -1 -> quality is unknown

status
------

.. code-block::

   (method) RangeFinder_State_ud:status()
     -> integer

 get sensor status

voltage
-------

.. code-block::

   (method) RangeFinder_State_ud:voltage()
     -> integer

 get voltage in millivolts, if applicable, otherwise 0

----

SRV_Channels
============

 desc

.. code-block::

   table

----

SRV_Channels.find_channel
=========================

 Returns first servo output number (zero indexed) of an output assigned output_function (See SERVOx_FUNCTION parameters ). 0 = SERVO1_FUNCTION ect. Nil if none is assigned.

@\ *param* ``function_num`` — servo function (See SERVOx_FUNCTION parameters)

@\ *return* — output channel number if available

.. code-block::

   (method) SRV_Channels:find_channel(function_num: integer)
     -> integer|nil

----

SRV_Channels.get_emergency_stop
===============================

 Get emergency stop state if active no motors of any kind will be active

.. code-block::

   return #1:
       | true -- E-Stop active
       | false -- E-Stop inactive

.. code-block::

   (method) SRV_Channels:get_emergency_stop()
     -> boolean

----

SRV_Channels.get_output_pwm
===========================

 Returns first servo output PWM value an output assigned output_function (See SERVOx_FUNCTION parameters). Nil if none is assigned.

@\ *param* ``function_num`` — servo function (See SERVOx_FUNCTION parameters)

@\ *return* — output pwm if available

.. code-block::

   (method) SRV_Channels:get_output_pwm(function_num: integer)
     -> integer|nil

----

SRV_Channels.get_output_scaled
==============================

 Get the scaled value for a given servo function

@\ *param* ``function_num`` — servo function (See SERVOx_FUNCTION parameters)

@\ *return* — scaled value

.. code-block::

   (method) SRV_Channels:get_output_scaled(function_num: integer)
     -> number

----

SRV_Channels.get_safety_state
=============================

 Get safety state

.. code-block::

   return #1:
       | true -- Disarmed outputs inactive
       | false -- Armed outputs live

.. code-block::

   (method) SRV_Channels:get_safety_state()
     -> boolean

----

SRV_Channels.set_angle
======================

 desc

@\ *param* ``function_num`` — servo function (See SERVOx_FUNCTION parameters)

.. code-block::

   (method) SRV_Channels:set_angle(function_num: integer, angle: integer)

----

SRV_Channels.set_output_norm
============================

 desc

@\ *param* ``function_num`` — servo function (See SERVOx_FUNCTION parameters)

.. code-block::

   (method) SRV_Channels:set_output_norm(function_num: integer, value: number)

----

SRV_Channels.set_output_pwm
===========================

 Set the pwm for a given servo output function

@\ *param* ``function_num`` — servo function number (See SERVOx_FUNCTION parameters)

@\ *param* ``pwm`` — pwm value

.. code-block::

   (method) SRV_Channels:set_output_pwm(function_num: integer, pwm: integer)

----

SRV_Channels.set_output_pwm_chan
================================

 Set the pwm for a given servo output channel

@\ *param* ``chan`` — servo channel number (zero indexed)

@\ *param* ``pwm`` — pwm value

.. code-block::

   (method) SRV_Channels:set_output_pwm_chan(chan: integer, pwm: integer)

----

SRV_Channels.set_output_pwm_chan_timeout
========================================

 Sets servo channel to specified PWM for a time in ms. This overrides any commands from the autopilot until the timeout expires.

@\ *param* ``chan`` — servo channel number (zero indexed)

@\ *param* ``pwm`` — pwm value

@\ *param* ``timeout_ms`` — duration of the override

.. code-block::

   (method) SRV_Channels:set_output_pwm_chan_timeout(chan: integer, pwm: integer, timeout_ms: integer)

----

SRV_Channels.set_output_scaled
==============================

 Set the scaled value of the output function, scale is out of the value set with the set_range or set_angle call

@\ *param* ``function_num`` — servo function (See SERVOx_FUNCTION parameters)

@\ *param* ``value`` — scaled value

.. code-block::

   (method) SRV_Channels:set_output_scaled(function_num: integer, value: number)

----

SRV_Channels.set_range
======================

 desc

@\ *param* ``function_num`` — servo function (See SERVOx_FUNCTION parameters)

.. code-block::

   (method) SRV_Channels:set_range(function_num: integer, range: integer)

----

ScriptingCANBuffer_ud
=====================

 desc

add_filter
----------

.. code-block::

   (method) ScriptingCANBuffer_ud:add_filter(mask: number|uint32_t_ud, value: number|uint32_t_ud)
     -> boolean

 Add a filter to the CAN buffer, mask is bitwise ANDed with the frame id and compared to value if not match frame is not buffered
 By default no filters are added and all frames are buffered, write is not affected by filters
 Maximum number of filters is 8

@\ *return* — returns true if the filler was added successfully

read_frame
----------

.. code-block::

   (method) ScriptingCANBuffer_ud:read_frame()
     -> CANFrame_ud|nil

 desc

write_frame
-----------

.. code-block::

   (method) ScriptingCANBuffer_ud:write_frame(frame: CANFrame_ud, timeout_us: number|uint32_t_ud)
     -> boolean

 desc

----

Socket
======

 Get a new socket

.. code-block::

   function Socket(datagram: boolean)
     -> SocketAPM_ud

----

SocketAPM_ud
============

 network socket class

accept
------

.. code-block::

   (method) SocketAPM_ud:accept()
     -> SocketAPM_ud|nil

accept new incoming sockets, returning a new socket.

.. code-block::

    Must be used on a stream socket in listen state

--

bind
----

.. code-block::

   (method) SocketAPM_ud:bind(IP_address: string, port: integer)
     -> boolean

 bind to an address. Use "0.0.0.0" for wildcard bind

close
-----

.. code-block::

   (method) SocketAPM_ud:close()

close a socket. Note that there is no automatic garbage
   collection of sockets so you must close a socket when you are

   finished with it or you will run out of sockets
--------------------------------------------------

connect
-------

.. code-block::

   (method) SocketAPM_ud:connect(IP_address: string, port: integer)
     -> boolean

 connect a socket to an endpoint

is_connected
------------

.. code-block::

   (method) SocketAPM_ud:is_connected()
     -> boolean

 return true if a socket is connected

listen
------

.. code-block::

   (method) SocketAPM_ud:listen(backlog: integer)
     -> boolean

 setup a socket to listen

pollin
------

.. code-block::

   (method) SocketAPM_ud:pollin(timeout_ms: number|uint32_t_ud)
     -> boolean

 check for available input

pollout
-------

.. code-block::

   (method) SocketAPM_ud:pollout(timeout_ms: number|uint32_t_ud)
     -> boolean

 check for availability of space to write to socket

recv
----

.. code-block::

   (method) SocketAPM_ud:recv(length: integer)
     -> string|nil

 receive data from a socket

reuseaddress
------------

.. code-block::

   (method) SocketAPM_ud:reuseaddress()
     -> boolean

 enable SO_REUSEADDR on a socket

send
----

.. code-block::

   (method) SocketAPM_ud:send(str: string, len: number|uint32_t_ud)
     -> integer

 send a lua string. May contain binary data

sendfile
--------

.. code-block::

   (method) SocketAPM_ud:sendfile(filehandle: string)
     -> boolean

setup to send all remaining data from a filehandle to the socket
   this also "closes" the socket and the file from the point of view of lua

   the underlying socket and file are both closed on end of file
----------------------------------------------------------------

@\ *return* — success

set_blocking
------------

.. code-block::

   (method) SocketAPM_ud:set_blocking(blocking: boolean)
     -> boolean

 set blocking state of socket

----

Vector2f
========

 Create Vector2f object

.. code-block::

   function Vector2f()
     -> Vector2f_ud

----

Vector2f_ud
===========

 Vector2f is a userdata object that holds a 2D vector with x and y components. The components are stored as floating point numbers.
 To create a new Vector2f you can call Vector2f() to allocate a new one, or call a method that returns one to you.

angle
-----

.. code-block::

   (method) Vector2f_ud:angle()
     -> number

 Calculate the angle of this vector in radians
 2PI + atan2(-x, y)

@\ *return* — angle in radians

copy
----

.. code-block::

   (method) Vector2f_ud:copy()
     -> Vector2f_ud

 Copy this Vector2f returning a new userdata object

@\ *return* — a copy of this Vector2f

is_inf
------

.. code-block::

   (method) Vector2f_ud:is_inf()
     -> boolean

 Check if either components of the vector are infinite

@\ *return* — true if either components are infinite

is_nan
------

.. code-block::

   (method) Vector2f_ud:is_nan()
     -> boolean

 Check if either components of the vector are nan

@\ *return* — true if either components are nan

is_zero
-------

.. code-block::

   (method) Vector2f_ud:is_zero()
     -> boolean

 Check if both components of the vector are zero

@\ *return* — true if both components are zero

length
------

.. code-block::

   (method) Vector2f_ud:length()
     -> number

 Calculate length of this vector sqrt(x^2 + y^2)

@\ *return* — length of this vector

normalize
---------

.. code-block::

   (method) Vector2f_ud:normalize()

 normalize this vector to a unit length

rotate
------

.. code-block::

   (method) Vector2f_ud:rotate(angle_rad: number)

 rotate vector by angle in radians

@\ *param* ``angle_rad`` — angle in radians

x
-

.. code-block::

   (method) Vector2f_ud:x()
     -> number

 get x component

y
-

.. code-block::

   (method) Vector2f_ud:y()
     -> number

 get y component

----

Vector3f
========

 Create Vector3f object

.. code-block::

   function Vector3f()
     -> Vector3f_ud

----

Vector3f_ud
===========

 Vector3f is a userdata object that holds a 3D vector with x, y and z components.
 The components are stored as floating point numbers.
 To create a new Vector3f you can call Vector3f() to allocate a new one, or call a method that returns one to you.

angle
-----

.. code-block::

   (method) Vector3f_ud:angle(v2: Vector3f_ud)
     -> number

 Computes angle between this vector and vector v2

copy
----

.. code-block::

   (method) Vector3f_ud:copy()
     -> Vector3f_ud

 Copy this Vector3f returning a new userdata object

@\ *return* — a copy of this Vector3f

cross
-----

.. code-block::

   (method) Vector3f_ud:cross(vector: Vector3f_ud)
     -> Vector3f_ud

 Cross product of two Vector3fs

@\ *return* — result

dot
---

.. code-block::

   (method) Vector3f_ud:dot(vector: Vector3f_ud)
     -> number

 Dot product of two Vector3fs

@\ *return* — result

is_inf
------

.. code-block::

   (method) Vector3f_ud:is_inf()
     -> boolean

 Check if any components of the vector are infinite

@\ *return* — true if any components are infinite

is_nan
------

.. code-block::

   (method) Vector3f_ud:is_nan()
     -> boolean

 Check if any components of the vector are nan

@\ *return* — true if any components are nan

is_zero
-------

.. code-block::

   (method) Vector3f_ud:is_zero()
     -> boolean

 Check if all components of the vector are zero

@\ *return* — true if all components are zero

length
------

.. code-block::

   (method) Vector3f_ud:length()
     -> number

 Calculate length of this vector sqrt(x^2 + y^2 + z^2)

@\ *return* — length of this vector

normalize
---------

.. code-block::

   (method) Vector3f_ud:normalize()

 normalize this vector to a unit length

rotate_xy
---------

.. code-block::

   (method) Vector3f_ud:rotate_xy(param1: number)

 Rotate vector by angle in radians in xy plane leaving z untouched

@\ *param* ``param1`` — XY rotation in radians

scale
-----

.. code-block::

   (method) Vector3f_ud:scale(scale_factor: number)
     -> Vector3f_ud

 Return a new Vector3 based on this one with scaled length and the same changing direction

@\ *return* — scaled copy of this vector

x
-

.. code-block::

   (method) Vector3f_ud:x()
     -> number

 get x component

xy
--

.. code-block::

   (method) Vector3f_ud:xy()
     -> Vector2f_ud

 return the x and y components of this vector as a Vector2f

y
-

.. code-block::

   (method) Vector3f_ud:y()
     -> number

 get y component

z
-

.. code-block::

   (method) Vector3f_ud:z()
     -> number

 get z component

----

ahrs
====

 The ahrs library represents the Attitude Heading Reference System computed by the autopilot. 
 It provides estimates for the vehicles attitude, and position.

.. code-block::

   table

----

ahrs.airspeed_estimate
======================

 Return the estimated airspeed of the vehicle if available

@\ *return* — airspeed in meters / second if available

.. code-block::

   (method) ahrs:airspeed_estimate()
     -> number|nil

----

ahrs.body_to_earth
==================

 desc

.. code-block::

   (method) ahrs:body_to_earth(vector: Vector3f_ud)
     -> Vector3f_ud

----

ahrs.earth_to_body
==================

 desc

.. code-block::

   (method) ahrs:earth_to_body(vector: Vector3f_ud)
     -> Vector3f_ud

----

ahrs.get_EAS2TAS
================

 desc

.. code-block::

   (method) ahrs:get_EAS2TAS()
     -> number

----

ahrs.get_accel
==============

 desc

.. code-block::

   (method) ahrs:get_accel()
     -> Vector3f_ud

----

ahrs.get_gyro
=============

 Returns a Vector3f containing the current smoothed and filtered gyro rates (in radians/second)

@\ *return* — roll, pitch, yaw gyro rates in radians / second

.. code-block::

   (method) ahrs:get_gyro()
     -> Vector3f_ud

----

ahrs.get_hagl
=============

  Returns nil, or the latest altitude estimate above ground level in meters

@\ *return* — height above ground level in meters

.. code-block::

   (method) ahrs:get_hagl()
     -> number|nil

----

ahrs.get_home
=============

 Returns a Location that contains the vehicles current home waypoint.

@\ *return* — home location

.. code-block::

   (method) ahrs:get_home()
     -> Location_ud

----

ahrs.get_location
=================

 Returns nil or Location userdata that contains the vehicles current position.
 Note: This will only return a Location if the system considers the current estimate to be reasonable.

@\ *return* — current location if available

.. code-block::

   (method) ahrs:get_location()
     -> Location_ud|nil

----

ahrs.get_origin
===============

 desc

.. code-block::

   (method) ahrs:get_origin()
     -> Location_ud|nil

----

ahrs.get_pitch
==============

 Returns the current vehicle euler pitch angle in radians.

@\ *return* — pitch angle in radians.

.. code-block::

   (method) ahrs:get_pitch()
     -> number

----

ahrs.get_position
=================

 same as ``get_location`` will be removed

.. code-block::

   (method) ahrs:get_position()
     -> Location_ud|nil

----

ahrs.get_posvelyaw_source_set
=============================

 desc

.. code-block::

   (method) ahrs:get_posvelyaw_source_set()
     -> integer

----

ahrs.get_quaternion
===================

 desc

.. code-block::

   (method) ahrs:get_quaternion()
     -> Quaternion_ud|nil

----

ahrs.get_relative_position_D_home
=================================

 desc

.. code-block::

   (method) ahrs:get_relative_position_D_home()
     -> number

----

ahrs.get_relative_position_NED_home
===================================

 desc

.. code-block::

   (method) ahrs:get_relative_position_NED_home()
     -> Vector3f_ud|nil

----

ahrs.get_relative_position_NED_origin
=====================================

 desc

.. code-block::

   (method) ahrs:get_relative_position_NED_origin()
     -> Vector3f_ud|nil

----

ahrs.get_roll
=============

 Returns the current vehicle euler roll angle in radians.

@\ *return* — roll angle in radians

.. code-block::

   (method) ahrs:get_roll()
     -> number

----

ahrs.get_variances
==================

 desc

.. code-block::

   (method) ahrs:get_variances()
     -> number|nil
     2. number|nil
     3. number|nil
     4. Vector3f_ud|nil
     5. number|nil

----

ahrs.get_vel_innovations_and_variances_for_source
=================================================

 desc

.. code-block::

   (method) ahrs:get_vel_innovations_and_variances_for_source(source: integer)
     -> Vector3f_ud|nil
     2. Vector3f_ud|nil

----

ahrs.get_velocity_NED
=====================

 Returns nil, or a Vector3f containing the current NED vehicle velocity in meters/second in north, east, and down components.

@\ *return* — North, east, down velcoity in meters / second if available

.. code-block::

   (method) ahrs:get_velocity_NED()
     -> Vector3f_ud|nil

----

ahrs.get_vibration
==================

 desc

.. code-block::

   (method) ahrs:get_vibration()
     -> Vector3f_ud

----

ahrs.get_yaw
============

 Returns the current vehicle euler yaw angle in radians.

@\ *return* — yaw angle in radians.

.. code-block::

   (method) ahrs:get_yaw()
     -> number

----

ahrs.groundspeed_vector
=======================

 Get current groundspeed vector in meter / second

@\ *return* — ground speed vector, North East, meters / second

.. code-block::

   (method) ahrs:groundspeed_vector()
     -> Vector2f_ud

----

ahrs.head_wind
==============

 Forward head-wind component in m/s. Negative means tail-wind

.. code-block::

   (method) ahrs:head_wind()
     -> number

----

ahrs.healthy
============

 desc

.. code-block::

   (method) ahrs:healthy()
     -> boolean

----

ahrs.home_is_set
================

 Returns a true if home position has been set.

@\ *return* — true if home position has been set

.. code-block::

   (method) ahrs:home_is_set()
     -> boolean

----

ahrs.initialised
================

 desc

.. code-block::

   (method) ahrs:initialised()
     -> boolean

----

ahrs.set_home
=============

 desc

.. code-block::

   (method) ahrs:set_home(loc: Location_ud)
     -> boolean

----

ahrs.set_origin
===============

 desc

.. code-block::

   (method) ahrs:set_origin(loc: Location_ud)
     -> boolean

----

ahrs.set_posvelyaw_source_set
=============================

 desc

.. code-block::

   (method) ahrs:set_posvelyaw_source_set(source_set_idx: integer)

----

ahrs.wind_alignment
===================

 Determine how aligned heading_deg is with the wind. Return result
 is 1.0 when perfectly aligned heading into wind, -1 when perfectly
 aligned with-wind, and zero when perfect cross-wind. There is no
 distinction between a left or right cross-wind. Wind speed is ignored

.. code-block::

   (method) ahrs:wind_alignment(heading_deg: number)
     -> number

----

ahrs.wind_estimate
==================

 Returns a Vector3f containing the current wind estimate for the vehicle.

@\ *return* — wind estiamte North, East, Down meters / second

.. code-block::

   (method) ahrs:wind_estimate()
     -> Vector3f_ud

----

analog
======

 desc

.. code-block::

   table

----

analog.channel
==============

 desc

.. code-block::

   (method) analog:channel()
     -> AP_HAL__AnalogSource_ud|nil

----

arming
======

 The Arming library provides access to arming status and commands.

.. code-block::

   table

----

arming.arm
==========

 Attempts to arm the vehicle. Returns true if successful.

@\ *return* — true if armed successfully

.. code-block::

   (method) arming:arm()
     -> boolean

----

arming.disarm
=============

 Disarms the vehicle in all cases. Returns false only if already disarmed.

@\ *return* — true if disarmed successfully, false if already disarmed.

.. code-block::

   (method) arming:disarm()
     -> boolean

----

arming.get_aux_auth_id
======================

 desc

.. code-block::

   (method) arming:get_aux_auth_id()
     -> integer|nil

----

arming.is_armed
===============

 Returns a true if vehicle is currently armed.

@\ *return* — true if armed

.. code-block::

   (method) arming:is_armed()
     -> boolean

----

arming.pre_arm_checks
=====================

 desc

.. code-block::

   (method) arming:pre_arm_checks()
     -> boolean

----

arming.set_aux_auth_failed
==========================

 desc

.. code-block::

   (method) arming:set_aux_auth_failed(auth_id: integer, fail_msg: string)

----

arming.set_aux_auth_passed
==========================

 desc

.. code-block::

   (method) arming:set_aux_auth_passed(auth_id: integer)

----

attitude_control
================

 desc

.. code-block::

   table

----

attitude_control.set_forward_enable
===================================

 desc

.. code-block::

   (method) attitude_control:set_forward_enable(bool: boolean)

----

attitude_control.set_lateral_enable
===================================

 desc

.. code-block::

   (method) attitude_control:set_lateral_enable(bool: boolean)

----

attitude_control.set_offset_roll_pitch
======================================

 desc

.. code-block::

   (method) attitude_control:set_offset_roll_pitch(roll_deg: number, pitch_deg: number)

----

baro
====

 desc

.. code-block::

   table

----

baro.get_altitude
=================

 get current altitude in meters relative to altitude at the time
 of the last calibrate() call, typically at boot

.. code-block::

   (method) baro:get_altitude()
     -> number

----

baro.get_external_temperature
=============================

 get external temperature in degrees C

@\ *return* — temperature in degrees C

.. code-block::

   (method) baro:get_external_temperature()
     -> number

----

baro.get_pressure
=================

 Returns pressure in Pascal. Divide by 100 for millibars or hectopascals

@\ *return* — pressure in Pascal

.. code-block::

   (method) baro:get_pressure()
     -> number

----

baro.get_temperature
====================

 get temperature in degrees C

@\ *return* — temperature in degrees C

.. code-block::

   (method) baro:get_temperature()
     -> number

----

baro.healthy
============

 Check if a baro sensor is healthy

@\ *param* ``instance`` — the 0-based index of the BARO instance to return.

.. code-block::

   (method) baro:healthy(instance: integer)
     -> boolean

----

battery
=======

 The battery library provides access to information about the currently connected batteries on the vehicle.

.. code-block::

   table

----

battery.capacity_remaining_pct
==============================

 Returns the remaining percentage of battery (from 0 to 100), or nil if energy monitoring is not available.

@\ *param* ``instance`` — battery instance

@\ *return* — remaining capacity as a percentage of total capacity if available

.. code-block::

   (method) battery:capacity_remaining_pct(instance: integer)
     -> integer|nil

----

battery.consumed_mah
====================

 Returns the capacity (in milliamp hours) used from the battery, or nil if current monitoring is not available.

@\ *param* ``instance`` — battery instance

@\ *return* — consumed capacity in milliamp hours

.. code-block::

   (method) battery:consumed_mah(instance: integer)
     -> number|nil

----

battery.consumed_wh
===================

 Returns the used watt hours from the battery, or nil if energy monitoring is not available.

@\ *param* ``instance`` — battery instance

@\ *return* — consumed energy in watt hours if available

.. code-block::

   (method) battery:consumed_wh(instance: integer)
     -> number|nil

----

battery.current_amps
====================

 Returns the current (in Amps) that is currently being consumed by the battery, or nil if current monitoring is not available.

@\ *param* ``instance`` — battery instance

@\ *return* — current in amps if available

.. code-block::

   (method) battery:current_amps(instance: integer)
     -> number|nil

----

battery.get_cell_voltage
========================

 get individual cell voltage

.. code-block::

   (method) battery:get_cell_voltage(instance: integer, cell: integer)
     -> number|nil

----

battery.get_cycle_count
=======================

 Returns cycle count of the battery or nil if not available.

@\ *param* ``instance`` — battery instance

@\ *return* — cycle count if available

.. code-block::

   (method) battery:get_cycle_count(instance: integer)
     -> integer|nil

----

battery.get_temperature
=======================

 Returns the temperature of the battery in degrees Celsius if the battery supports temperature monitoring.

@\ *param* ``instance`` — battery instance

@\ *return* — temperature if available

.. code-block::

   (method) battery:get_temperature(instance: integer)
     -> number|nil

----

battery.handle_scripting
========================

 desc

.. code-block::

   (method) battery:handle_scripting(idx: integer, state: BattMonitorScript_State_ud)
     -> boolean

----

battery.has_failsafed
=====================

 Returns true if any of the batteries being monitored have triggered a failsafe.

@\ *return* — true if any battery has failsafed

.. code-block::

   (method) battery:has_failsafed()
     -> boolean

----

battery.healthy
===============

 Returns true if the requested battery instance is healthy. Healthy is considered to be ArduPilot is currently able to monitor the battery.

@\ *param* ``instance`` — battery instance

.. code-block::

   (method) battery:healthy(instance: integer)
     -> boolean

----

battery.num_instances
=====================

 Returns the number of battery instances currently available.

@\ *return* — number of instances

.. code-block::

   (method) battery:num_instances()
     -> integer

----

battery.overpower_detected
==========================

 returns true if too much power is being drawn from the battery being monitored.

@\ *param* ``instance`` — battery instance

@\ *return* — true if in overpower condition

.. code-block::

   (method) battery:overpower_detected(instance: integer)
     -> boolean

----

battery.pack_capacity_mah
=========================

 Returns the full pack capacity (in milliamp hours) from the battery.

@\ *param* ``instance`` — battery instance

@\ *return* — capacity in milliamp hours

.. code-block::

   (method) battery:pack_capacity_mah(instance: integer)
     -> integer

----

battery.reset_remaining
=======================

 desc

@\ *param* ``instance`` — battery instance

.. code-block::

   (method) battery:reset_remaining(instance: integer, percentage: number)
     -> boolean

----

battery.voltage
===============

 Returns the voltage of the selected battery instance.

@\ *param* ``instance`` — battery instance

@\ *return* — voltage

.. code-block::

   (method) battery:voltage(instance: integer)
     -> number

----

battery.voltage_resting_estimate
================================

 Returns the estimated battery voltage if it was not under load.

@\ *param* ``instance`` — battery instance

@\ *return* — resting voltage

.. code-block::

   (method) battery:voltage_resting_estimate(instance: integer)
     -> number

----

button
======

 button handling

.. code-block::

   table

----

button.get_button_state
=======================

 Returns button state if available. Buttons are 1 indexed.

@\ *param* ``button_number`` — button number 1 indexed.

.. code-block::

   (method) button:get_button_state(button_number: integer)
     -> boolean

----

camera
======

 desc

.. code-block::

   table

----

camera.get_state
================

 desc

.. code-block::

   (method) camera:get_state(instance: integer)
     -> AP_Camera__camera_state_t_ud|nil

----

camera.record_video
===================

 desc

.. code-block::

   (method) camera:record_video(instance: integer, start_recording: boolean)
     -> boolean

----

camera.set_trigger_distance
===========================

 desc

.. code-block::

   (method) camera:set_trigger_distance(instance: integer, distance_m: number)

----

camera.take_picture
===================

 desc

.. code-block::

   (method) camera:take_picture(instance: integer)

----

compass
=======

 desc

.. code-block::

   table

----

compass.healthy
===============

 Check if the compass is healthy

@\ *param* ``instance`` — the 0-based index of the compass instance to return.

.. code-block::

   (method) compass:healthy(instance: integer)
     -> boolean

----

dirlist
=======

 desc

@\ *return* — table of filenames

@\ *return* — error string if fails

.. code-block::

   function dirlist(directoryname: string)
     -> table|nil
     2. string|nil

----

efi
===

 desc

.. code-block::

   table

----

efi.get_backend
===============

 desc

.. code-block::

   (method) efi:get_backend(instance: integer)
     -> AP_EFI_Backend_ud|nil

----

efi.get_state
=============

 desc

.. code-block::

   (method) efi:get_state()
     -> EFI_State_ud

----

esc_telem
=========

 desc

.. code-block::

   table

----

esc_telem.get_consumption_mah
=============================

 desc

@\ *param* ``instance`` — esc instance 0 indexed

.. code-block::

   (method) esc_telem:get_consumption_mah(instance: integer)
     -> number|nil

----

esc_telem.get_current
=====================

 desc

@\ *param* ``instance`` — esc instance 0 indexed

.. code-block::

   (method) esc_telem:get_current(instance: integer)
     -> number|nil

----

esc_telem.get_motor_temperature
===============================

 desc

@\ *param* ``instance`` — esc instance 0 indexed

.. code-block::

   (method) esc_telem:get_motor_temperature(instance: integer)
     -> integer|nil

----

esc_telem.get_rpm
=================

 desc

@\ *param* ``instance`` — esc instance 0 indexed

.. code-block::

   (method) esc_telem:get_rpm(instance: integer)
     -> number|nil

----

esc_telem.get_temperature
=========================

 desc

@\ *param* ``instance`` — esc instance 0 indexed

.. code-block::

   (method) esc_telem:get_temperature(instance: integer)
     -> integer|nil

----

esc_telem.get_usage_seconds
===========================

 Returns an individual ESC’s usage time in seconds, or nil if not available.

@\ *param* ``instance`` — esc instance 0 indexed

@\ *return* — usage time in seconds, nill if not available.

.. code-block::

   (method) esc_telem:get_usage_seconds(instance: integer)
     -> uint32_t_ud|nil

----

esc_telem.get_voltage
=====================

 desc

@\ *param* ``instance`` — esc instance 0 indexed

.. code-block::

   (method) esc_telem:get_voltage(instance: integer)
     -> number|nil

----

esc_telem.set_rpm_scale
=======================

 set scale factor for RPM on a motor

@\ *param* ``esc_index`` — esc instance 0 indexed

@\ *param* ``scale_factor`` — factor

.. code-block::

   (method) esc_telem:set_rpm_scale(esc_index: integer, scale_factor: number)

----

esc_telem.update_rpm
====================

 update RPM for an ESC

@\ *param* ``esc_index`` — esc instance 0 indexed

@\ *param* ``rpm`` — RPM

@\ *param* ``error_rate`` — error rate

.. code-block::

   (method) esc_telem:update_rpm(esc_index: integer, rpm: integer, error_rate: number)

----

esc_telem.update_telem_data
===========================

 update telemetry data for an ESC instance

@\ *param* ``instance`` — esc instance 0 indexed

@\ *param* ``data_mask`` — bit mask of what fields are filled in

.. code-block::

   (method) esc_telem:update_telem_data(instance: integer, telemdata: ESCTelemetryData_ud, data_mask: integer)

----

fence
=====

 Geofence library

.. code-block::

   table

----

fence.get_breach_time
=====================

 Returns the time at which the current breach started

@\ *return* ``system_time`` — milliseconds

.. code-block::

   (method) fence:get_breach_time()
     -> system_time: uint32_t_ud

----

fence.get_breaches
==================

 Returns the type bitmask of any breached fences

.. code-block::

   return #1:
       | 1 -- Maximim altitude
       | 2 -- Circle
       | 4 -- Polygon
       | 8 -- Minimum altitude

.. code-block::

   (method) fence:get_breaches()
     -> integer|1|2|4|8

----

follow
======

 desc

.. code-block::

   table

----

follow.get_last_update_ms
=========================

 desc

.. code-block::

   (method) follow:get_last_update_ms()
     -> uint32_t_ud

----

follow.get_target_heading_deg
=============================

 desc

.. code-block::

   (method) follow:get_target_heading_deg()
     -> number|nil

----

follow.get_target_location_and_velocity
=======================================

 desc

.. code-block::

   (method) follow:get_target_location_and_velocity()
     -> Location_ud|nil
     2. Vector3f_ud|nil

----

follow.get_target_location_and_velocity_ofs
===========================================

 desc

.. code-block::

   (method) follow:get_target_location_and_velocity_ofs()
     -> Location_ud|nil
     2. Vector3f_ud|nil

----

follow.have_target
==================

 desc

.. code-block::

   (method) follow:have_target()
     -> boolean

----

frsky_sport
===========

 desc

.. code-block::

   table

----

frsky_sport.prep_number
=======================

 desc

.. code-block::

   (method) frsky_sport:prep_number(number: integer, digits: integer, power: integer)
     -> integer

----

frsky_sport.sport_telemetry_push
================================

 desc

.. code-block::

   (method) frsky_sport:sport_telemetry_push(sensor: integer, frame: integer, appid: integer, data: integer)
     -> boolean

----

fs
==

 desc

.. code-block::

   table

----

fs.crc32
========

 Get crc32 checksum of a file with given name

.. code-block::

   (method) fs:crc32(file_name: string)
     -> uint32_t_ud|nil

----

fs.format
=========

 Format the SD card. This is a async operation, use get_format_status to get the status of the format

.. code-block::

   (method) fs:format()
     -> boolean

----

fs.get_format_status
====================

 Get the current status of a format. 0=NOT_STARTED, 1=PENDING, 2=IN_PROGRESS, 3=SUCCESS, 4=FAILURE

.. code-block::

   (method) fs:get_format_status()
     -> integer

----

fs.stat
=======

 desc

.. code-block::

   (method) fs:stat(param1: string)
     -> stat_t_ud|nil

----

gcs
===

 MAVLink interaction with ground control station

.. code-block::

   table

----

gcs.enable_high_latency_connections
===================================

 set high latency control state. Analogous to MAV_CMD_CONTROL_HIGH_LATENCY

@\ *param* ``enabled`` — true to enable or false to disable

.. code-block::

   (method) gcs:enable_high_latency_connections(enabled: boolean)

----

gcs.frame_type
==============

 get the vehicle MAV_TYPE

.. code-block::

   return #1:
       | '0' -- MAV_TYPE_GENERIC=0, /* Generic micro air vehicle | */
       | '1' -- MAV_TYPE_FIXED_WING=1, /* Fixed wing aircraft. | */
       | '2' -- MAV_TYPE_QUADROTOR=2, /* Quadrotor | */
       | '3' -- MAV_TYPE_COAXIAL=3, /* Coaxial helicopter | */
       | '4' -- MAV_TYPE_HELICOPTER=4, /* Normal helicopter with tail rotor. | */
       | '5' -- MAV_TYPE_ANTENNA_TRACKER=5, /* Ground installation | */
       | '6' -- MAV_TYPE_GCS=6, /* Operator control unit / ground control station | */
       | '7' -- MAV_TYPE_AIRSHIP=7, /* Airship, controlled | */
       | '8' -- MAV_TYPE_FREE_BALLOON=8, /* Free balloon, uncontrolled | */
       | '9' -- MAV_TYPE_ROCKET=9, /* Rocket | */
       | '10' -- MAV_TYPE_GROUND_ROVER=10, /* Ground rover | */
       | '11' -- MAV_TYPE_SURFACE_BOAT=11, /* Surface vessel, boat, ship | */
       | '12' -- MAV_TYPE_SUBMARINE=12, /* Submarine | */
       | '13' -- MAV_TYPE_HEXAROTOR=13, /* Hexarotor | */
       | '14' -- MAV_TYPE_OCTOROTOR=14, /* Octorotor | */
       | '15' -- MAV_TYPE_TRICOPTER=15, /* Tricopter | */
       | '16' -- MAV_TYPE_FLAPPING_WING=16, /* Flapping wing | */
       | '17' -- MAV_TYPE_KITE=17, /* Kite | */
       | '18' -- MAV_TYPE_ONBOARD_CONTROLLER=18, /* Onboard companion controller | */
       | '19' -- MAV_TYPE_VTOL_DUOROTOR=19, /* Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter. | */
       | '20' -- MAV_TYPE_VTOL_QUADROTOR=20, /* Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter. | */
       | '21' -- MAV_TYPE_VTOL_TILTROTOR=21, /* Tiltrotor VTOL | */
       | '22' -- MAV_TYPE_VTOL_RESERVED2=22, /* VTOL reserved 2 | */
       | '23' -- MAV_TYPE_VTOL_RESERVED3=23, /* VTOL reserved 3 | */
       | '24' -- MAV_TYPE_VTOL_RESERVED4=24, /* VTOL reserved 4 | */
       | '25' -- MAV_TYPE_VTOL_RESERVED5=25, /* VTOL reserved 5 | */
       | '26' -- MAV_TYPE_GIMBAL=26, /* Gimbal | */
       | '27' -- MAV_TYPE_ADSB=27, /* ADSB system | */
       | '28' -- MAV_TYPE_PARAFOIL=28, /* Steerable, nonrigid airfoil | */
       | '29' -- MAV_TYPE_DODECAROTOR=29, /* Dodecarotor | */
       | '30' -- MAV_TYPE_CAMERA=30, /* Camera | */
       | '31' -- MAV_TYPE_CHARGING_STATION=31, /* Charging station | */
       | '32' -- MAV_TYPE_FLARM=32, /* FLARM collision avoidance system | */
       | '33' -- MAV_TYPE_SERVO=33, /* Servo | */
       | '34' -- MAV_TYPE_ODID=34, /* Open Drone ID. See https://mavlink.io/en/services/opendroneid.html. | */
       | '35' -- MAV_TYPE_DECAROTOR=35, /* Decarotor | */
       | '36' -- MAV_TYPE_BATTERY=36, /* Battery | */
       | '37' -- MAV_TYPE_PARACHUTE=37, /* Parachute | */
       | '38' -- MAV_TYPE_LOG=38, /* Log | */
       | '39' -- MAV_TYPE_OSD=39, /* OSD | */
       | '40' -- MAV_TYPE_IMU=40, /* IMU | */
       | '41' -- MAV_TYPE_GPS=41, /* GPS | */
       | '42' -- MAV_TYPE_WINCH=42, /* Winch | */
       | '43' -- MAV_TYPE_ENUM_END=43, /*  | */

.. code-block::

   (method) gcs:frame_type()
     -> integer|'0'|'1'|'10'|'11'...(+40)

----

gcs.get_high_latency_status
===========================

 get the the current state of high latency control

.. code-block::

   (method) gcs:get_high_latency_status()
     -> boolean

----

gcs.get_hud_throttle
====================

 get the throttle value in %

.. code-block::

   (method) gcs:get_hud_throttle()
     -> integer

----

gcs.last_seen
=============

 Return the system time when a gcs with id of SYSID_MYGCS was last seen

@\ *return* — system time in milliseconds

.. code-block::

   (method) gcs:last_seen()
     -> uint32_t_ud

----

gcs.send_named_float
====================

 send named float value using NAMED_VALUE_FLOAT message

@\ *param* ``name`` — up to 10 chars long

@\ *param* ``value`` — value to send

.. code-block::

   (method) gcs:send_named_float(name: string, value: number)

----

gcs.send_text
=============

 send text with severity level

.. code-block::

   severity:
       | '0' -- Emergency: System is unusable. This is a "panic" condition.
       | '1' -- Alert: Action should be taken immediately. Indicates error in non-critical systems.
       | '2' -- Critical: Action must be taken immediately. Indicates failure in a primary system.
       | '3' -- Error: Indicates an error in secondary/redundant systems.
       | '4' -- Warning: Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
       | '5' -- Notice: An unusual event has occurred, though not an error condition. This should be investigated for the root cause.
       | '6' -- Info: Normal operational messages. Useful for logging. No action is required for these messages.
       | '7' -- Debug: Useful non-operational messages that can assist in debugging. These should not occur during normal operation.

.. code-block::

   (method) gcs:send_text(severity: integer|'0'|'1'|'2'|'3'...(+4), text: string)

----

gcs.set_message_interval
========================

 set message interval for a given serial port and message id

@\ *param* ``port_num`` — serial port number

@\ *param* ``msg_id`` — MAVLink message id

@\ *param* ``interval_us`` — interval in micro seconds

.. code-block::

   return #1:
       | '0' -- Accepted
       | '4' -- Failed

.. code-block::

   (method) gcs:set_message_interval(port_num: integer, msg_id: number|uint32_t_ud, interval_us: integer)
     -> integer|'0'|'4'

----

gpio
====

 Control of general purpose input/output pins

.. code-block::

   table

----

gpio.pinMode
============

 set GPIO pin mode

.. code-block::

   mode:
       | '0' -- input
       | '1' -- output

.. code-block::

   (method) gpio:pinMode(pin_number: integer, mode: integer|'0'|'1')

----

gpio.read
=========

 read GPIO input

@\ *return* — pin state

.. code-block::

   (method) gpio:read(pin_number: integer)
     -> boolean

----

gpio.toggle
===========

 toggle GPIO output

.. code-block::

   (method) gpio:toggle(pin_number: integer)

----

gpio.write
==========

 write GPIO output

.. code-block::

   value:
       | '0' -- low
       | '1' -- high

.. code-block::

   (method) gpio:write(pin_number: integer, value: integer|'0'|'1')

----

gps
===

 The GPS library provides access to information about the GPS’s on the vehicle.

.. code-block::

   table

----

gps.GPS_OK_FIX_2D
=================

.. code-block::

   integer

----

gps.GPS_OK_FIX_3D
=================

.. code-block::

   integer

----

gps.GPS_OK_FIX_3D_DGPS
======================

.. code-block::

   integer

----

gps.GPS_OK_FIX_3D_RTK_FIXED
===========================

.. code-block::

   integer

----

gps.GPS_OK_FIX_3D_RTK_FLOAT
===========================

.. code-block::

   integer

----

gps.NO_FIX
==========

.. code-block::

   integer

----

gps.NO_GPS
==========

.. code-block::

   integer

----

gps.first_unconfigured_gps
==========================

  Returns nil or the instance number of the first GPS that has not been fully configured. If all GPS’s have been configured this returns nil.

.. code-block::

   (method) gps:first_unconfigured_gps()
     -> integer|nil

----

gps.get_antenna_offset
======================

 Returns a Vector3f that contains the offsets of the GPS in meters in the body frame.

@\ *param* ``instance`` — instance number

@\ *return* — anteena offset vector forward, right, down in meters

.. code-block::

   (method) gps:get_antenna_offset(instance: integer)
     -> Vector3f_ud

----

gps.get_hdop
============

 Returns the horizontal dilution of precision of the GPS instance.

@\ *param* ``instance`` — instance number

@\ *return* — hdop

.. code-block::

   (method) gps:get_hdop(instance: integer)
     -> integer

----

gps.get_vdop
============

 Returns the vertical dilution of precision of the GPS instance.

@\ *param* ``instance`` — instance number

@\ *return* — vdop

.. code-block::

   (method) gps:get_vdop(instance: integer)
     -> integer

----

gps.gps_yaw_deg
===============

 get yaw from GPS in degrees

@\ *param* ``instance`` — instance number

@\ *return* — yaw in degrees

@\ *return* — yaw accuracy in degrees

@\ *return* — time in milliseconds of last yaw reading

.. code-block::

   (method) gps:gps_yaw_deg(instance: integer)
     -> number|nil
     2. number|nil
     3. uint32_t_ud|nil

----

gps.ground_course
=================

 Returns the ground course of the vehicle in degrees. You must check the status to know if the ground course is still current.

@\ *param* ``instance`` — instance number

@\ *return* — ground course in degrees

.. code-block::

   (method) gps:ground_course(instance: integer)
     -> number

----

gps.ground_speed
================

 Returns the ground speed of the vehicle in meters per second. You must check the status to know if the ground speed is still current.

@\ *param* ``instance`` — instance number

@\ *return* — ground speed m/s

.. code-block::

   (method) gps:ground_speed(instance: integer)
     -> number

----

gps.have_vertical_velocity
==========================

 Returns true if the GPS instance can report the vertical velocity.

@\ *param* ``instance`` — instance number

@\ *return* — true if vertical velocity is available

.. code-block::

   (method) gps:have_vertical_velocity(instance: integer)
     -> boolean

----

gps.horizontal_accuracy
=======================

 horizontal RMS accuracy estimate in m

@\ *param* ``instance`` — instance number

@\ *return* — accuracy in meters

.. code-block::

   (method) gps:horizontal_accuracy(instance: integer)
     -> number|nil

----

gps.last_fix_time_ms
====================

 Returns the time of the last fix in system milliseconds.

@\ *param* ``instance`` — instance number

@\ *return* — system time of last fix in milliseconds

.. code-block::

   (method) gps:last_fix_time_ms(instance: integer)
     -> uint32_t_ud

----

gps.last_message_time_ms
========================

 desc

@\ *param* ``instance`` — instance number

.. code-block::

   (method) gps:last_message_time_ms(instance: integer)
     -> uint32_t_ud

----

gps.location
============

 eturns a Location userdata for the last GPS position. You must check the status to know if the location is still current, if it is NO_GPS, or NO_FIX then it will be returning old data.

@\ *param* ``instance`` — instance number

@\ *return* — gps location

.. code-block::

   (method) gps:location(instance: integer)
     -> Location_ud

----

gps.num_sats
============

 Returns the number of satellites that the GPS is currently tracking.

@\ *param* ``instance`` — instance number

@\ *return* — number of satellites

.. code-block::

   (method) gps:num_sats(instance: integer)
     -> integer

----

gps.num_sensors
===============

 Returns the number of connected GPS devices.
 If GPS blending is turned on that will show up as the third sensor, and be reported here.

@\ *return* — number of sensors

.. code-block::

   (method) gps:num_sensors()
     -> integer

----

gps.primary_sensor
==================

 Returns which GPS is currently being used as the primary GPS device.

@\ *return* — primary sensor instance

.. code-block::

   (method) gps:primary_sensor()
     -> integer

----

gps.speed_accuracy
==================

 Returns nil, or the speed accuracy of the GPS in meters per second, if the information is available for the GPS instance.

@\ *param* ``instance`` — instance number

@\ *return* — 3D velocity RMS accuracy estimate in m/s if available

.. code-block::

   (method) gps:speed_accuracy(instance: integer)
     -> number|nil

----

gps.status
==========

 Returns the GPS fix status. Compare this to one of the GPS fix types.
 Posible status are provided as values on the gps object. eg: gps.GPS_OK_FIX_3D

@\ *param* ``instance`` — instance number

@\ *return* — status

.. code-block::

   (method) gps:status(instance: integer)
     -> integer

----

gps.time_week
=============

 Returns the GPS week number.

@\ *param* ``instance`` — instance number

@\ *return* — week number

.. code-block::

   (method) gps:time_week(instance: integer)
     -> integer

----

gps.time_week_ms
================

 Returns the number of milliseconds into the current week.

@\ *param* ``instance`` — instance number

@\ *return* — milliseconds of current week

.. code-block::

   (method) gps:time_week_ms(instance: integer)
     -> uint32_t_ud

----

gps.velocity
============

 Returns a Vector3f that contains the velocity as observed by the GPS.
 You must check the status to know if the velocity is still current.

@\ *param* ``instance`` — instance number

@\ *return* — 3D velocity in m/s, in NED format

.. code-block::

   (method) gps:velocity(instance: integer)
     -> Vector3f_ud

----

gps.vertical_accuracy
=====================

 desc

@\ *param* ``instance`` — instance number

.. code-block::

   (method) gps:vertical_accuracy(instance: integer)
     -> number|nil

----

i2c
===

 i2c bus interaction

.. code-block::

   table

----

i2c.get_device
==============

 get a i2c device handler

@\ *param* ``bus`` — bus number

@\ *param* ``address`` — device address 0 to 128

@\ *param* ``clock`` — optional bus clock, default 400000

@\ *param* ``smbus`` — optional sumbus flag, default false

.. code-block::

   (method) i2c:get_device(bus: integer, address: integer, clock?: number|uint32_t_ud, smbus?: boolean)
     -> AP_HAL__I2CDevice_ud|nil

----

ins
===

 desc

.. code-block::

   table

----

ins.accels_consistent
=====================

 Check if the accelerometers are consistent

@\ *param* ``threshold`` — the threshold allowed before returning false

.. code-block::

   (method) ins:accels_consistent(threshold: number)
     -> boolean

----

ins.calibrating
===============

 Get if the INS is currently calibrating

.. code-block::

   (method) ins:calibrating()
     -> boolean

----

ins.get_accel
=============

 Get the value of a specific accelerometer

@\ *param* ``instance`` — the 0-based index of the accelerometer instance to return.

.. code-block::

   (method) ins:get_accel(instance: integer)
     -> Vector3f_ud

----

ins.get_accel_health
====================

 Check if a specific accelerometer sensor is healthy

@\ *param* ``instance`` — the 0-based index of the accelerometer instance to return.

.. code-block::

   (method) ins:get_accel_health(instance: integer)
     -> boolean

----

ins.get_gyro
============

 Get the value of a specific gyroscope

@\ *param* ``instance`` — the 0-based index of the gyroscope instance to return.

.. code-block::

   (method) ins:get_gyro(instance: integer)
     -> Vector3f_ud

----

ins.get_gyro_health
===================

 Check if a specific gyroscope sensor is healthy

@\ *param* ``instance`` — the 0-based index of the gyroscope instance to return.

.. code-block::

   (method) ins:get_gyro_health(instance: integer)
     -> boolean

----

ins.get_temperature
===================

 desc

.. code-block::

   (method) ins:get_temperature(instance: integer)
     -> number

----

ins.gyros_consistent
====================

 Check if the gyrometers are consistent

@\ *param* ``threshold`` — the allowed threshold in degrees per second

.. code-block::

   (method) ins:gyros_consistent(threshold: integer)
     -> boolean

----

iomcu
=====

 desc

.. code-block::

   table

----

iomcu.healthy
=============

 Check if the IO is healthy

.. code-block::

   (method) iomcu:healthy()
     -> boolean

----

logger
======

 data flash logging to SD card

.. code-block::

   table

----

logger.log_file_content
=======================

 log a files content to onboard log

@\ *param* ``filename`` — file name

.. code-block::

   (method) logger:log_file_content(filename: string)

----

logger.write
============

 write value to data flash log with given types and names with units and multipliers, timestamp will be automatically added

@\ *param* ``name`` — up to 4 characters

@\ *param* ``labels`` — comma separated value labels, up to 58 characters

@\ *param* ``format`` — type format string, see https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/README.md

@\ *param* ``units`` — units string

@\ *param* ``multipliers`` — multipliers string

@\ *param* ``...`` — data to be logged, type to match format string

.. code-block::

   (method) logger:write(name: string, labels: string, format: string, units: string, multipliers: string, ...string|number|uint32_t_ud)

.. code-block::

   (method) logger:write(name: string, labels: string, format: string, ...string|number|uint32_t_ud)

----

mavlink
=======

 desc

.. code-block::

   table

----

mavlink.block_command
=====================

 Block a given MAV_CMD from being procceced by ArduPilot

.. code-block::

   (method) mavlink:block_command(comand_id: integer)
     -> boolean

----

mavlink.init
============

 initializes mavlink

.. code-block::

   (method) mavlink:init(num_rx_msgid: number|uint32_t_ud, msg_queue_length: number|uint32_t_ud)

----

mavlink.receive_chan
====================

 receives mavlink message marked for receive using mavlink:register_rx_msgid

@\ *return* — bytes

@\ *return* — mavlink channel

@\ *return* — receive_timestamp

.. code-block::

   (method) mavlink:receive_chan()
     -> string
     2. number
     3. uint32_t_ud

----

mavlink.register_rx_msgid
=========================

 marks mavlink message for receive, message id can be get using mavlink_msgs.get_msgid("MSG_NAME")

@\ *return* — false if id has been registered already

.. code-block::

   (method) mavlink:register_rx_msgid(msg_id: number)
     -> boolean

----

mavlink.send_chan
=================

 sends mavlink message, to use this function the call should be like this:
 mavlink:send(chan, mavlink_msgs.encode("MSG_NAME", {param1 = value1, param2 = value2, ...}})

@\ *return* — success

.. code-block::

   (method) mavlink:send_chan(chan: integer, msgid: integer, message: string)
     -> boolean

----

mavlink_mission_item_int_t
==========================

.. code-block::

   function mavlink_mission_item_int_t()
     -> mavlink_mission_item_int_t_ud

----

mavlink_mission_item_int_t_ud
=============================

 desc

command
-------

.. code-block::

   (method) mavlink_mission_item_int_t_ud:command()
     -> integer

 get field

current
-------

.. code-block::

   (method) mavlink_mission_item_int_t_ud:current()
     -> integer

 get field

frame
-----

.. code-block::

   (method) mavlink_mission_item_int_t_ud:frame()
     -> integer

 get field

param1
------

.. code-block::

   (method) mavlink_mission_item_int_t_ud:param1()
     -> number

 get field

param2
------

.. code-block::

   (method) mavlink_mission_item_int_t_ud:param2()
     -> number

 get field

param3
------

.. code-block::

   (method) mavlink_mission_item_int_t_ud:param3()
     -> number

 get field

param4
------

.. code-block::

   (method) mavlink_mission_item_int_t_ud:param4()
     -> number

 get field

seq
---

.. code-block::

   (method) mavlink_mission_item_int_t_ud:seq()
     -> integer

 get field

x
-

.. code-block::

   (method) mavlink_mission_item_int_t_ud:x()
     -> integer

 get field

y
-

.. code-block::

   (method) mavlink_mission_item_int_t_ud:y()
     -> integer

 get field

z
-

.. code-block::

   (method) mavlink_mission_item_int_t_ud:z()
     -> number

 get field

----

micros
======

 system time in microseconds

@\ *return* — microseconds

.. code-block::

   function micros()
     -> uint32_t_ud

----

millis
======

 system time in milliseconds

@\ *return* — milliseconds

.. code-block::

   function millis()
     -> uint32_t_ud

----

mission
=======

 desc

.. code-block::

   table

----

mission.MISSION_COMPLETE
========================

.. code-block::

   integer

----

mission.MISSION_RUNNING
=======================

.. code-block::

   integer

----

mission.MISSION_STOPPED
=======================

.. code-block::

   integer

----

mission.clear
=============

 clear - clears out mission

.. code-block::

   (method) mission:clear()
     -> boolean

----

mission.cmd_has_location
========================

 returns true if the mission cmd has a location

.. code-block::

   (method) mission:cmd_has_location(cmd: integer)
     -> boolean

----

mission.get_current_do_cmd_id
=============================

 get_current_do_cmd_id - returns id of the active "do" command

.. code-block::

   (method) mission:get_current_do_cmd_id()
     -> integer

----

mission.get_current_nav_id
==========================

 get_current_nav_id - return the id of the current nav command

.. code-block::

   (method) mission:get_current_nav_id()
     -> integer

----

mission.get_current_nav_index
=============================

 get_current_nav_index - returns the current "navigation" command index
 Note that this will return 0 if there is no command. This is
 used in MAVLink reporting of the mission command

.. code-block::

   (method) mission:get_current_nav_index()
     -> integer

----

mission.get_index_of_jump_tag
=============================

 desc

.. code-block::

   (method) mission:get_index_of_jump_tag(tag: integer)
     -> integer

----

mission.get_item
================

 get any WP items in any order in a mavlink-ish kinda way.

.. code-block::

   (method) mission:get_item(index: integer)
     -> mavlink_mission_item_int_t_ud|nil

----

mission.get_last_jump_tag
=========================

 Jump Tags. When a JUMP_TAG is run in the mission, either via DO_JUMP_TAG or
 by just being the next item, the tag is remembered and the age is set to 1.
 Only the most recent tag is remembered. It's age is how many NAV items have
 progressed since the tag was seen. While executing the tag, the
 age will be 1. The next NAV command after it will tick the age to 2, and so on.

.. code-block::

   (method) mission:get_last_jump_tag()
     -> integer|nil
     2. integer|nil

----

mission.get_prev_nav_cmd_id
===========================

 get_prev_nav_cmd_id - returns the previous "navigation" command id
     if there was no previous nav command it returns AP_MISSION_CMD_ID_NONE (0)
      we do not return the entire command to save on RAM

.. code-block::

   (method) mission:get_prev_nav_cmd_id()
     -> integer

----

mission.jump_to_abort_landing_sequence
======================================

 Jump to the landing abort sequence

.. code-block::

   (method) mission:jump_to_abort_landing_sequence()
     -> boolean

----

mission.jump_to_landing_sequence
================================

 Jump the mission to the start of the closest landing sequence. Returns true if one was found

.. code-block::

   (method) mission:jump_to_landing_sequence()
     -> boolean

----

mission.jump_to_tag
===================

 Set the mission index to the first JUMP_TAG with this tag.
 Returns true on success, else false if no appropriate JUMP_TAG match can be found or if setting the index failed

.. code-block::

   (method) mission:jump_to_tag(tag: integer)
     -> boolean

----

mission.num_commands
====================

 num_commands - returns total number of commands in the mission
                 this number includes offset 0, the home location

.. code-block::

   (method) mission:num_commands()
     -> integer

----

mission.set_current_cmd
=======================

 set_current_cmd - jumps to command specified by index

.. code-block::

   (method) mission:set_current_cmd(index: integer)
     -> boolean

----

mission.set_item
================

 set any WP items in any order in a mavlink-ish kinda way.

.. code-block::

   (method) mission:set_item(index: integer, item: mavlink_mission_item_int_t_ud)
     -> boolean

----

mission.state
=============

 status - returns the status of the mission (i.e. Mission_Started, Mission_Complete, Mission_Stopped

.. code-block::

   (method) mission:state()
     -> integer

----

mission_receive
===============

 receive mission command from running mission

@\ *return* — command start time milliseconds

@\ *return* — command param 1

@\ *return* — command param 2

@\ *return* — command param 3

@\ *return* — command param 4

.. code-block::

   function mission_receive()
     -> uint32_t_ud|nil
     2. integer|nil
     3. number|nil
     4. number|nil
     5. number|nil

----

motor_factor_table
==================

.. code-block::

   function motor_factor_table()
     -> motor_factor_table_ud

----

motor_factor_table_ud
=====================

 desc

pitch
-----

.. code-block::

   (method) motor_factor_table_ud:pitch(index: integer)
     -> number

 get array field

roll
----

.. code-block::

   (method) motor_factor_table_ud:roll(index: integer)
     -> number

 get array field

throttle
--------

.. code-block::

   (method) motor_factor_table_ud:throttle(index: integer)
     -> number

 get array field

yaw
---

.. code-block::

   (method) motor_factor_table_ud:yaw(index: integer)
     -> number

 get array field

----

motors
======

 desc

.. code-block::

   table

----

motors.get_desired_spool_state
==============================

 desc

.. code-block::

   (method) motors:get_desired_spool_state()
     -> integer

----

motors.get_forward
==================

 get forward motor output

.. code-block::

   (method) motors:get_forward()
     -> number

----

motors.get_interlock
====================

 Get motors interlock state, the state of motors controlled by AP_Motors, Copter and Quadplane VTOL motors. Not plane forward flight motors.

.. code-block::

   return #1:
       | true -- motors active
       | false -- motors inactive

.. code-block::

   (method) motors:get_interlock()
     -> boolean

----

motors.get_lateral
==================

 get lateral motor output

.. code-block::

   (method) motors:get_lateral()
     -> number

----

motors.get_pitch
================

 get pitch P+I+D out

.. code-block::

   (method) motors:get_pitch()
     -> number

----

motors.get_pitch_ff
===================

 get pitch FF out

.. code-block::

   (method) motors:get_pitch_ff()
     -> number

----

motors.get_roll
===============

 get roll P+I+D

.. code-block::

   (method) motors:get_roll()
     -> number

----

motors.get_roll_ff
==================

 get roll FF out

.. code-block::

   (method) motors:get_roll_ff()
     -> number

----

motors.get_spool_state
======================

 get throttle motor output

.. code-block::

   return #1:
       | '0' -- Shut down
       | '1' -- Ground idle
       | '2' -- Spooling up
       | '3' -- Throttle unlimited
       | '4' -- Spooling down

.. code-block::

   (method) motors:get_spool_state()
     -> integer|'0'|'1'|'2'|'3'...(+1)

----

motors.get_throttle
===================

 get throttle motor output

.. code-block::

   (method) motors:get_throttle()
     -> number

----

motors.get_yaw
==============

 get yaw P+I+D

.. code-block::

   (method) motors:get_yaw()
     -> number

----

motors.get_yaw_ff
=================

 get yaw FF output

.. code-block::

   (method) motors:get_yaw_ff()
     -> number

----

motors.set_external_limits
==========================

 set external limit flags for each axis to prevent integrator windup

.. code-block::

   (method) motors:set_external_limits(roll: boolean, pitch: boolean, yaw: boolean, throttle_lower: boolean, throttle_upper: boolean)

----

motors.set_frame_string
=======================

 desc

.. code-block::

   (method) motors:set_frame_string(param1: string)

----

mount
=====

 desc

.. code-block::

   table

----

mount.get_angle_target
======================

 desc

@\ *return* — roll_deg

@\ *return* — pitch_deg

@\ *return* — yaw_deg

@\ *return* — yaw_is_earth_frame

.. code-block::

   (method) mount:get_angle_target(instance: integer)
     -> number|nil
     2. number|nil
     3. number|nil
     4. boolean|nil

----

mount.get_attitude_euler
========================

 desc

@\ *return* — roll_deg

@\ *return* — pitch_deg

@\ *return* — yaw_bf_deg

.. code-block::

   (method) mount:get_attitude_euler(instance: integer)
     -> number|nil
     2. number|nil
     3. number|nil

----

mount.get_location_target
=========================

 desc

.. code-block::

   (method) mount:get_location_target(instance: integer)
     -> Location_ud|nil

----

mount.get_mode
==============

 desc

.. code-block::

   (method) mount:get_mode(instance: integer)
     -> integer

----

mount.get_rate_target
=====================

 desc

@\ *return* — roll_degs

@\ *return* — pitch_degs

@\ *return* — yaw_degs

@\ *return* — yaw_is_earth_frame

.. code-block::

   (method) mount:get_rate_target(instance: integer)
     -> number|nil
     2. number|nil
     3. number|nil
     4. boolean|nil

----

mount.set_angle_target
======================

 desc

.. code-block::

   (method) mount:set_angle_target(instance: integer, roll_deg: number, pitch_deg: number, yaw_deg: number, yaw_is_earth_frame: boolean)

----

mount.set_attitude_euler
========================

 desc

.. code-block::

   (method) mount:set_attitude_euler(instance: integer, roll_deg: number, pitch_deg: number, yaw_deg: number)

----

mount.set_mode
==============

 desc

.. code-block::

   (method) mount:set_mode(instance: integer, mode: integer)

----

mount.set_rate_target
=====================

 desc

.. code-block::

   (method) mount:set_rate_target(instance: integer, roll_degs: number, pitch_degs: number, yaw_degs: number, yaw_is_earth_frame: boolean)

----

mount.set_roi_target
====================

 desc

.. code-block::

   (method) mount:set_roi_target(instance: integer, target_loc: Location_ud)

----

networking
==========

 desc

.. code-block::

   table

----

networking.address_to_str
=========================

 conver uint32_t address to string

.. code-block::

   (method) networking:address_to_str(ip4addr: number|uint32_t_ud)
     -> string

----

networking.get_gateway_active
=============================

 desc

.. code-block::

   (method) networking:get_gateway_active()
     -> uint32_t_ud

----

networking.get_ip_active
========================

 desc

.. code-block::

   (method) networking:get_ip_active()
     -> uint32_t_ud

----

networking.get_netmask_active
=============================

 desc

.. code-block::

   (method) networking:get_netmask_active()
     -> uint32_t_ud

----

notify
======

 desc

.. code-block::

   table

----

notify.handle_rgb
=================

 desc

.. code-block::

   (method) notify:handle_rgb(red: integer, green: integer, blue: integer, rate_hz: integer)

----

notify.handle_rgb_id
====================

 desc

.. code-block::

   (method) notify:handle_rgb_id(red: integer, green: integer, blue: integer, id: integer)

----

notify.play_tune
================

 Plays a MML tune through the buzzer on the vehicle. The tune is provided as a string.
 An online tune tester can be found here: https://firmware.ardupilot.org/Tools/ToneTester/

.. code-block::

   (method) notify:play_tune(tune: string)

----

notify.release_text
===================

 desc

.. code-block::

   (method) notify:release_text(row: integer)

----

notify.send_text
================

 Display text on a notify display, text too long to fit will automatically be scrolled.

@\ *param* ``text`` — upto 50 characters

@\ *param* ``row`` — row number to display on, 0 is at the top.

.. code-block::

   (method) notify:send_text(text: string, row: integer)

----

onvif
=====

 desc

.. code-block::

   table

----

onvif.get_pan_tilt_limit_max
============================

 desc

.. code-block::

   (method) onvif:get_pan_tilt_limit_max()
     -> Vector2f_ud

----

onvif.get_pan_tilt_limit_min
============================

 desc

.. code-block::

   (method) onvif:get_pan_tilt_limit_min()
     -> Vector2f_ud

----

onvif.set_absolutemove
======================

 desc

.. code-block::

   (method) onvif:set_absolutemove(pan: number, tilt: number, zoom: number)
     -> boolean

----

onvif.start
===========

 desc

.. code-block::

   (method) onvif:start(username: string, password: string, httphostname: string)
     -> boolean

----

optical_flow
============

 desc

.. code-block::

   table

----

optical_flow.enabled
====================

 desc

.. code-block::

   (method) optical_flow:enabled()
     -> boolean

----

optical_flow.healthy
====================

 desc

.. code-block::

   (method) optical_flow:healthy()
     -> boolean

----

optical_flow.quality
====================

 desc

.. code-block::

   (method) optical_flow:quality()
     -> integer

----

param
=====

 Parameter access

.. code-block::

   table

----

param.add_param
===============

 desc

.. code-block::

   (method) param:add_param(table_key: integer, param_num: integer, name: string, default_value: number)
     -> boolean

----

param.add_table
===============

 desc

.. code-block::

   (method) param:add_table(table_key: integer, prefix: string, num_params: integer)
     -> boolean

----

param.get
=========

 Get parameter value

@\ *param* ``name`` — parameter name

@\ *return* — nill if parameter was not found

.. code-block::

   (method) param:get(name: string)
     -> number|nil

----

param.set
=========

 set parameter value, this will not be retained over a reboot

@\ *param* ``name`` — parameter name

@\ *param* ``value`` — value to set

@\ *return* — true if parameter was found

.. code-block::

   (method) param:set(name: string, value: number)
     -> boolean

----

param.set_and_save
==================

 set and save parameter value, this will be saved for subsequent boots

@\ *param* ``name`` — parameter name

@\ *param* ``value`` — value to set and save

@\ *return* — true if parameter was found

.. code-block::

   (method) param:set_and_save(name: string, value: number)
     -> boolean

----

param.set_default
=================

 Set default value for a given parameter. If the parameter has not been configured by the user then the set to this default value.

@\ *param* ``name`` — parameter name

@\ *param* ``value`` — default value

@\ *return* — true if parameter was found

.. code-block::

   (method) param:set_default(name: string, value: number)
     -> boolean

----

periph
======

 desc

.. code-block::

   table

----

periph.can_printf
=================

 desc

.. code-block::

   (method) periph:can_printf(text: string)

----

periph.get_vehicle_state
========================

 desc

.. code-block::

   (method) periph:get_vehicle_state()
     -> uint32_t_ud

----

periph.get_yaw_earth
====================

 desc

.. code-block::

   (method) periph:get_yaw_earth()
     -> number

----

periph.reboot
=============

 desc

.. code-block::

   (method) periph:reboot(hold_in_bootloader: boolean)

----

precland
========

 precision landing access

.. code-block::

   table

----

precland.get_last_valid_target_ms
=================================

 get the time of the last valid target

.. code-block::

   (method) precland:get_last_valid_target_ms()
     -> uint32_t_ud

----

precland.get_target_location
============================

 get Location of target or nil if target not acquired

.. code-block::

   (method) precland:get_target_location()
     -> Location_ud|nil

----

precland.get_target_velocity
============================

 get NE velocity of target or nil if not available

.. code-block::

   (method) precland:get_target_velocity()
     -> Vector2f_ud|nil

----

precland.healthy
================

 return true if precland system is healthy

.. code-block::

   (method) precland:healthy()
     -> boolean

----

precland.target_acquired
========================

 return true if target is acquired

.. code-block::

   (method) precland:target_acquired()
     -> boolean

----

print
=====

 Print text, if MAVLink is available the value will be sent with debug severity
 If no MAVLink the value will be sent over can
 equivalent to gcs:send_text(7, text) or periph:can_printf(text)

.. code-block::

   function print(text: string|number)

----

proximity
=========

 desc

.. code-block::

   table

----

proximity.get_backend
=====================

 get backend based on proximity instance provided

.. code-block::

   (method) proximity:get_backend(instance: integer)
     -> AP_Proximity_Backend_ud|nil

----

proximity.get_closest_object
============================

 desc

.. code-block::

   (method) proximity:get_closest_object()
     -> number|nil
     2. number|nil

----

proximity.get_object_angle_and_distance
=======================================

 desc

.. code-block::

   (method) proximity:get_object_angle_and_distance(object_number: integer)
     -> number|nil
     2. number|nil

----

proximity.get_object_count
==========================

 desc

.. code-block::

   (method) proximity:get_object_count()
     -> integer

----

proximity.get_status
====================

 desc

.. code-block::

   (method) proximity:get_status()
     -> integer

----

proximity.num_sensors
=====================

 desc

.. code-block::

   (method) proximity:num_sensors()
     -> integer

----

quadplane
=========

 desc

.. code-block::

   table

----

quadplane.abort_landing
=======================

 abort a VTOL landing, climbing back up

.. code-block::

   (method) quadplane:abort_landing()
     -> boolean

----

quadplane.in_assisted_flight
============================

 desc

.. code-block::

   (method) quadplane:in_assisted_flight()
     -> boolean

----

quadplane.in_vtol_land_descent
==============================

 true in descent phase of VTOL landing

.. code-block::

   (method) quadplane:in_vtol_land_descent()
     -> boolean

----

quadplane.in_vtol_mode
======================

 desc

.. code-block::

   (method) quadplane:in_vtol_mode()
     -> boolean

----

rangefinder
===========

 desc

.. code-block::

   table

----

rangefinder.distance_cm_orient
==============================

 desc

.. code-block::

   (method) rangefinder:distance_cm_orient(orientation: integer)
     -> integer

----

rangefinder.get_backend
=======================

 get backend based on rangefinder instance provided

.. code-block::

   (method) rangefinder:get_backend(rangefinder_instance: integer)
     -> AP_RangeFinder_Backend_ud|nil

----

rangefinder.get_pos_offset_orient
=================================

 desc

.. code-block::

   (method) rangefinder:get_pos_offset_orient(orientation: integer)
     -> Vector3f_ud

----

rangefinder.ground_clearance_cm_orient
======================================

 desc

.. code-block::

   (method) rangefinder:ground_clearance_cm_orient(orientation: integer)
     -> integer

----

rangefinder.has_data_orient
===========================

 desc

.. code-block::

   (method) rangefinder:has_data_orient(orientation: integer)
     -> boolean

----

rangefinder.has_orientation
===========================

 desc

.. code-block::

   (method) rangefinder:has_orientation(orientation: integer)
     -> boolean

----

rangefinder.max_distance_cm_orient
==================================

 desc

.. code-block::

   (method) rangefinder:max_distance_cm_orient(orientation: integer)
     -> integer

----

rangefinder.min_distance_cm_orient
==================================

 desc

.. code-block::

   (method) rangefinder:min_distance_cm_orient(orientation: integer)
     -> integer

----

rangefinder.num_sensors
=======================

 desc

.. code-block::

   (method) rangefinder:num_sensors()
     -> integer

----

rangefinder.signal_quality_pct_orient
=====================================

 Current distance measurement signal quality for range finder at this orientation

.. code-block::

   (method) rangefinder:signal_quality_pct_orient(orientation: integer)
     -> integer

----

rangefinder.status_orient
=========================

 desc

.. code-block::

   (method) rangefinder:status_orient(orientation: integer)
     -> integer

----

rc
==

 desc

.. code-block::

   table

----

rc.find_channel_for_option
==========================

 desc

.. code-block::

   (method) rc:find_channel_for_option(aux_fun: integer)
     -> RC_Channel_ud|nil

----

rc.get_aux_cached
=================

 return cached level of aux function

.. code-block::

   (method) rc:get_aux_cached(aux_fn: integer)
     -> integer|nil

----

rc.get_channel
==============

 desc

.. code-block::

   (method) rc:get_channel(chan_num: integer)
     -> RC_Channel_ud|nil

----

rc.get_pwm
==========

 Returns the RC input PWM value given a channel number. Note that channel here is indexed from 1. Returns nill if channel is not available.

@\ *param* ``chan_num`` — input channel number, 1 indexed

@\ *return* — pwm input or nil if not availables

.. code-block::

   (method) rc:get_pwm(chan_num: integer)
     -> integer|nil

----

rc.has_valid_input
==================

 desc

.. code-block::

   (method) rc:has_valid_input()
     -> boolean

----

rc.run_aux_function
===================

 desc

.. code-block::

   ch_flag:
       | '0' -- low
       | '1' -- middle
       | '2' -- high

.. code-block::

   (method) rc:run_aux_function(aux_fun: integer, ch_flag: integer|'0'|'1'|'2')
     -> boolean

----

relay
=====

 The relay library provides access to controlling relay outputs.

.. code-block::

   table

----

relay.enabled
=============

 Returns true if the requested relay is enabled.

@\ *param* ``instance`` — relay instance

.. code-block::

   (method) relay:enabled(instance: integer)
     -> boolean

----

relay.get
=========

 return state of a relay

@\ *param* ``instance`` — relay instance

@\ *return* — relay state

.. code-block::

   (method) relay:get(instance: integer)
     -> integer

----

relay.off
=========

 Turns the requested relay off.

@\ *param* ``instance`` — relay instance

.. code-block::

   (method) relay:off(instance: integer)

----

relay.on
========

 Turns the requested relay on.

@\ *param* ``instance`` — relay instance

.. code-block::

   (method) relay:on(instance: integer)

----

relay.toggle
============

 Toggles the requested relay from on to off or from off to on.

@\ *param* ``instance`` — relay instance

.. code-block::

   (method) relay:toggle(instance: integer)

----

remove
======

desc

@\ *return* — true on success

@\ *return* — error string

@\ *return* — error number

.. code-block::

   function remove(filename: string)
     -> boolean|nil
     2. string|nil
     3. integer

----

rtc
===

 desc

.. code-block::

   table

----

rtc.clock_s_to_date_fields
==========================

 break a time in seconds since 1970 to GMT date elements

@\ *return* — year 20xx

@\ *return* — month 0-11

@\ *return* — day 1-31

@\ *return* — hour 0-23

@\ *return* — min 0-60

@\ *return* — sec 0-60

@\ *return* — weekday 0-6, sunday is 0

.. code-block::

   (method) rtc:clock_s_to_date_fields(param1: number|uint32_t_ud)
     -> integer|nil
     2. integer|nil
     3. integer|nil
     4. integer|nil
     5. integer|nil
     6. integer|nil
     7. integer|nil

----

rtc.date_fields_to_clock_s
==========================

 return a time since 1970 in seconds from GMT date elements

@\ *param* ``year`` — 20xx

@\ *param* ``month`` — 0-11

@\ *param* ``day`` — 1-31

@\ *param* ``hour`` — 0-23

@\ *param* ``min`` — 0-60

@\ *param* ``sec`` — 0-60

.. code-block::

   (method) rtc:date_fields_to_clock_s(year: integer, month: integer, day: integer, hour: integer, min: integer, sec: integer)
     -> uint32_t_ud

----

scripting
=========

 desc

.. code-block::

   table

----

scripting.restart_all
=====================

 desc

.. code-block::

   (method) scripting:restart_all()

----

serial
======

 Serial ports

.. code-block::

   table

----

serial.find_serial
==================

 Returns the UART instance that allows connections from scripts (those with SERIALx_PROTOCOL = 28).
 For instance = 0, returns first such UART, second for instance = 1, and so on.
 If such an instance is not found, returns nil.

@\ *param* ``instance`` — the 0-based index of the UART instance to return.

@\ *return* — the requested UART instance available for scripting, or nil if none.

.. code-block::

   (method) serial:find_serial(instance: integer)
     -> AP_HAL__UARTDriver_ud|nil

----

serialLED
=========

 This library allows the control of RGB LED strings via an output reserved for scripting and selected by SERVOx_FUNCTION = 94 thru 109 (Script Out 1 thru 16)

.. code-block::

   table

----

serialLED.send
==============

 Send the configured RGB values to the LED string

@\ *param* ``chan`` — output number to which the leds are attached 1-16

@\ *return* — true if successful

.. code-block::

   (method) serialLED:send(chan: integer)
     -> boolean

----

serialLED.set_RGB
=================

  Set the data for LED_number on the string attached channel output

@\ *param* ``chan`` — output number to which the leds are attached 1-16

@\ *param* ``led_index`` — led number 0 index, -1 sets all

@\ *param* ``red`` — red value 0 to 255

@\ *param* ``green`` — green value 0 to 255

@\ *param* ``blue`` — blue value 0 to 255

@\ *return* — true if successful

.. code-block::

   (method) serialLED:set_RGB(chan: integer, led_index: integer, red: integer, green: integer, blue: integer)
     -> boolean

----

serialLED.set_num_neopixel
==========================

 Sets the number of LEDs in a neopixel string on a servo output.

@\ *param* ``chan`` — output number to which the leds are attached 1-16

@\ *param* ``num_leds`` — number of leds in the string

@\ *return* — true if successful

.. code-block::

   (method) serialLED:set_num_neopixel(chan: integer, num_leds: integer)
     -> boolean

----

serialLED.set_num_neopixel_rgb
==============================

 Sets the number of LEDs in a rgb neopixel string on a servo output.

@\ *param* ``chan`` — output number to which the leds are attached 1-16

@\ *param* ``num_leds`` — number of leds in the string

@\ *return* — true if successful

.. code-block::

   (method) serialLED:set_num_neopixel_rgb(chan: integer, num_leds: integer)
     -> boolean

----

serialLED.set_num_profiled
==========================

 Sets the number of LEDs in a profiled string on a servo output.

@\ *param* ``chan`` — output number to which the leds are attached 1-16

@\ *param* ``num_leds`` — number of leds in the string

@\ *return* — true if successful

.. code-block::

   (method) serialLED:set_num_profiled(chan: integer, num_leds: integer)
     -> boolean

----

stat_t
======

.. code-block::

   function stat_t()
     -> stat_t_ud

----

stat_t_ud
=========

 desc

atime
-----

.. code-block::

   (method) stat_t_ud:atime()
     -> uint32_t_ud

 get last access time in seconds

ctime
-----

.. code-block::

   (method) stat_t_ud:ctime()
     -> uint32_t_ud

 get creation time in seconds

is_directory
------------

.. code-block::

   (method) stat_t_ud:is_directory()
     -> boolean

 return true if this is a directory

mode
----

.. code-block::

   (method) stat_t_ud:mode()
     -> integer

 get file mode

mtime
-----

.. code-block::

   (method) stat_t_ud:mtime()
     -> uint32_t_ud

 get last modification time in seconds

size
----

.. code-block::

   (method) stat_t_ud:size()
     -> uint32_t_ud

 get file size in bytes

----

sub
===

 Sub singleton

.. code-block::

   table

----

sub.get_and_clear_button_count
==============================

 Get count of joystick button presses, then clear count

.. code-block::

   (method) sub:get_and_clear_button_count(index: integer)
     -> integer

----

sub.get_rangefinder_target_cm
=============================

 SURFTRAK mode: return the rangefinder target in cm

.. code-block::

   (method) sub:get_rangefinder_target_cm()
     -> number

----

sub.is_button_pressed
=====================

 Return true if joystick button is currently pressed

.. code-block::

   (method) sub:is_button_pressed(index: integer)
     -> boolean

----

sub.rangefinder_alt_ok
======================

 Return true if rangefinder is healthy, includes a check for good signal quality

.. code-block::

   (method) sub:rangefinder_alt_ok()
     -> boolean

----

sub.set_rangefinder_target_cm
=============================

 SURFTRAK mode: set the rangefinder target in cm, return true if successful

.. code-block::

   (method) sub:set_rangefinder_target_cm(new_target_cm: number)
     -> boolean

----

terrain
=======

 The terrain library provides access to checking heights against a terrain database.

.. code-block::

   table

----

terrain.TerrainStatusDisabled
=============================

.. code-block::

   integer

----

terrain.TerrainStatusOK
=======================

.. code-block::

   integer

----

terrain.TerrainStatusUnhealthy
==============================

.. code-block::

   integer

----

terrain.enabled
===============

 Returns true if terrain is enabled.

.. code-block::

   (method) terrain:enabled()
     -> boolean

----

terrain.height_above_terrain
============================

 Returns the height (in meters) that the vehicle is currently above the terrain, or returns nil if that is not available.
 If extrapolate is true then allow return of an extrapolated terrain altitude based on the last available data

@\ *return* — height above terrain in meters if available

.. code-block::

   (method) terrain:height_above_terrain(extrapolate: boolean)
     -> number|nil

----

terrain.height_amsl
===================

  Returns the terrain height (in meters) above mean sea level at the provided Location userdata, or returns nil if that is not available.

@\ *param* ``loc`` — location at which to lookup terrain

@\ *param* ``corrected`` — if true the terrain altitude should be correced based on the diffrence bettween the database and measured altitude at home

@\ *return* — amsl altitude of terrain at given locaiton in meters

.. code-block::

   (method) terrain:height_amsl(loc: Location_ud, corrected: boolean)
     -> number|nil

----

terrain.height_terrain_difference_home
======================================

 find difference between home terrain height and the terrain height at the current location in meters. A positive result means the terrain is higher than home.
 return false is terrain at the current location or at home location is not available
 If extrapolate is true then allow return of an extrapolated terrain altitude based on the last available data

@\ *return* — height difference in meters if available

.. code-block::

   (method) terrain:height_terrain_difference_home(extrapolate: boolean)
     -> number|nil

----

terrain.status
==============

 Returns the current status of the terrain. Compare this to one of the terrain statuses (terrain.TerrainStatusDisabled, terrain.TerrainStatusUnhealthy, terrain.TerrainStatusOK).

@\ *return* — terrain status

.. code-block::

   (method) terrain:status()
     -> integer

----

uint32_t
========

 create uint32_t_ud with optional value

.. code-block::

   function uint32_t(value?: number|uint32_t_ud)
     -> uint32_t_ud

----

uint32_t_ud
===========

tofloat
-------

.. code-block::

   (method) uint32_t_ud:tofloat()
     -> number

 Convert to number

toint
-----

.. code-block::

   (method) uint32_t_ud:toint()
     -> integer

 Convert to integer

----

vehicle
=======

 desc

.. code-block::

   table

----

vehicle.get_circle_radius
=========================

 desc

.. code-block::

   (method) vehicle:get_circle_radius()
     -> number|nil

----

vehicle.get_control_mode_reason
===============================

 desc

.. code-block::

   (method) vehicle:get_control_mode_reason()
     -> integer

----

vehicle.get_control_output
==========================

 desc

.. code-block::

   control_output:
       | '1' -- Roll
       | '2' -- Pitch
       | '3' -- Throttle
       | '4' -- Yaw
       | '5' -- Lateral
       | '6' -- MainSail
       | '7' -- WingSail
       | '8' -- Walking_Height

.. code-block::

   (method) vehicle:get_control_output(control_output: integer|'1'|'2'|'3'|'4'...(+4))
     -> number|nil

----

vehicle.get_likely_flying
=========================

 Returns true if the autopilot thinks it is flying. Not guaranteed to be accurate.

@\ *return* — true if likely flying

.. code-block::

   (method) vehicle:get_likely_flying()
     -> boolean

----

vehicle.get_mode
================

  Returns current vehicle mode by mode_number.

@\ *return* — mode number. Values for each vehcile type can be found here: https://mavlink.io/en/messages/ardupilotmega.html#PLANE_MODE

.. code-block::

   (method) vehicle:get_mode()
     -> integer

----

vehicle.get_pan_tilt_norm
=========================

 desc

.. code-block::

   (method) vehicle:get_pan_tilt_norm()
     -> number|nil
     2. number|nil

----

vehicle.get_steering_and_throttle
=================================

 desc

.. code-block::

   (method) vehicle:get_steering_and_throttle()
     -> number|nil
     2. number|nil

----

vehicle.get_target_location
===========================

 Get the current target location if available in current mode

@\ *return* — target location

.. code-block::

   (method) vehicle:get_target_location()
     -> Location_ud|nil

----

vehicle.get_time_flying_ms
==========================

 Returns time in milliseconds since the autopilot thinks it started flying, or zero if not currently flying.

@\ *return* — flying time in milliseconds

.. code-block::

   (method) vehicle:get_time_flying_ms()
     -> uint32_t_ud

----

vehicle.get_wp_bearing_deg
==========================

 desc

.. code-block::

   (method) vehicle:get_wp_bearing_deg()
     -> number|nil

----

vehicle.get_wp_crosstrack_error_m
=================================

 desc

.. code-block::

   (method) vehicle:get_wp_crosstrack_error_m()
     -> number|nil

----

vehicle.get_wp_distance_m
=========================

 desc

.. code-block::

   (method) vehicle:get_wp_distance_m()
     -> number|nil

----

vehicle.has_ekf_failsafed
=========================

 desc

.. code-block::

   (method) vehicle:has_ekf_failsafed()
     -> boolean

----

vehicle.is_landing
==================

 desc

.. code-block::

   (method) vehicle:is_landing()
     -> boolean

----

vehicle.is_taking_off
=====================

 desc

.. code-block::

   (method) vehicle:is_taking_off()
     -> boolean

----

vehicle.nav_script_time
=======================

 desc

@\ *return* — id

@\ *return* — cmd

@\ *return* — arg1

@\ *return* — arg2

@\ *return* — arg3

@\ *return* — arg4

.. code-block::

   (method) vehicle:nav_script_time()
     -> integer|nil
     2. integer|nil
     3. number|nil
     4. number|nil
     5. integer|nil
     6. integer|nil

----

vehicle.nav_script_time_done
============================

 desc

.. code-block::

   (method) vehicle:nav_script_time_done(param1: integer)

----

vehicle.nav_scripting_enable
============================

 desc

.. code-block::

   (method) vehicle:nav_scripting_enable(param1: integer)
     -> boolean

----

vehicle.reboot
==============

 desc

.. code-block::

   (method) vehicle:reboot(hold_in_bootloader: boolean)

----

vehicle.set_circle_rate
=======================

 desc

.. code-block::

   (method) vehicle:set_circle_rate(rate_dps: number)
     -> boolean

----

vehicle.set_desired_speed
=========================

 desc sets autopilot nav speed (Copter and Rover)

.. code-block::

   (method) vehicle:set_desired_speed(param1: number)
     -> boolean

----

vehicle.set_desired_turn_rate_and_speed
=======================================

 desc

.. code-block::

   (method) vehicle:set_desired_turn_rate_and_speed(param1: number, param2: number)
     -> boolean

----

vehicle.set_land_descent_rate
=============================

 override landing descent rate, times out in 1s

.. code-block::

   (method) vehicle:set_land_descent_rate(rate: number)
     -> boolean

----

vehicle.set_mode
================

 Attempts to change vehicle mode to mode_number. Returns true if successful, false if mode change is not successful.

@\ *param* ``mode_number`` — mode number values for each vehcile type can be found here: https://mavlink.io/en/messages/ardupilotmega.html#PLANE_MODE

@\ *return* — success

.. code-block::

   (method) vehicle:set_mode(mode_number: integer)
     -> boolean

----

vehicle.set_rudder_offset
=========================

 desc

.. code-block::

   (method) vehicle:set_rudder_offset(rudder_pct: number, run_yaw_rate_control: boolean)

----

vehicle.set_steering_and_throttle
=================================

 desc

.. code-block::

   (method) vehicle:set_steering_and_throttle(steering: number, throttle: number)
     -> boolean

----

vehicle.set_target_angle_and_climbrate
======================================

 desc

.. code-block::

   (method) vehicle:set_target_angle_and_climbrate(roll_deg: number, pitch_deg: number, yaw_deg: number, climb_rate_ms: number, use_yaw_rate: boolean, yaw_rate_degs: number)
     -> boolean

----

vehicle.set_target_location
===========================

 Set the target veicle location in a guided mode

@\ *param* ``target_loc`` — target location

@\ *return* — true on success

.. code-block::

   (method) vehicle:set_target_location(target_loc: Location_ud)
     -> boolean

----

vehicle.set_target_pos_NED
==========================

 desc

.. code-block::

   (method) vehicle:set_target_pos_NED(target_pos: Vector3f_ud, use_yaw: boolean, yaw_deg: number, use_yaw_rate: boolean, yaw_rate_degs: number, yaw_relative: boolean, terrain_alt: boolean)
     -> boolean

----

vehicle.set_target_posvel_NED
=============================

 desc

.. code-block::

   (method) vehicle:set_target_posvel_NED(target_pos: Vector3f_ud, target_vel: Vector3f_ud)
     -> boolean

----

vehicle.set_target_posvelaccel_NED
==================================

 desc

.. code-block::

   (method) vehicle:set_target_posvelaccel_NED(target_pos: Vector3f_ud, target_vel: Vector3f_ud, target_accel: Vector3f_ud, use_yaw: boolean, yaw_deg: number, use_yaw_rate: boolean, yaw_rate_degs: number, yaw_relative: boolean)
     -> boolean

----

vehicle.set_target_throttle_rate_rpy
====================================

 desc

@\ *param* ``param1`` — throttle percent

@\ *param* ``param2`` — roll rate deg/s

@\ *param* ``param3`` — pitch rate deg/s

@\ *param* ``param4`` — yaw rate deg/s

.. code-block::

   (method) vehicle:set_target_throttle_rate_rpy(param1: number, param2: number, param3: number, param4: number)

----

vehicle.set_target_velaccel_NED
===============================

 desc

.. code-block::

   (method) vehicle:set_target_velaccel_NED(target_vel: Vector3f_ud, target_accel: Vector3f_ud, use_yaw: boolean, yaw_deg: number, use_yaw_rate: boolean, yaw_rate_degs: number, yaw_relative: boolean)
     -> boolean

----

vehicle.set_target_velocity_NED
===============================

 Sets the target velocity using a Vector3f object in a guided mode.

@\ *param* ``vel_ned`` — North, East, Down meters / second

@\ *return* — true on success

.. code-block::

   (method) vehicle:set_target_velocity_NED(vel_ned: Vector3f_ud)
     -> boolean

----

vehicle.set_velocity_match
==========================

 desc

.. code-block::

   (method) vehicle:set_velocity_match(param1: Vector2f_ud)
     -> boolean

----

vehicle.start_takeoff
=====================

 Trigger a takeoff start if in a auto or guided mode. Not supported by all vehicles

@\ *param* ``alt`` — takeoff altitude in meters

@\ *return* — true on success

.. code-block::

   (method) vehicle:start_takeoff(alt: number)
     -> boolean

----

vehicle.update_target_location
==============================

 desc

@\ *param* ``current_target`` — current target, from get_target_location()

@\ *param* ``new_target`` — new target

.. code-block::

   (method) vehicle:update_target_location(current_target: Location_ud, new_target: Location_ud)
     -> boolean

----

visual_odom
===========

 visual odometry object

.. code-block::

   table

----

visual_odom.healthy
===================

 visual odometry health

.. code-block::

   (method) visual_odom:healthy()
     -> boolean

----

visual_odom.quality
===================

 visual odometry quality as a percentage from 1 to 100 or 0 if unknown

.. code-block::

   (method) visual_odom:quality()
     -> integer

----

winch
=====

 desc

.. code-block::

   table

----

winch.get_rate_max
==================

 desc

.. code-block::

   (method) winch:get_rate_max()
     -> number

----

winch.healthy
=============

 desc

.. code-block::

   (method) winch:healthy()
     -> boolean

----

winch.relax
===========

 desc

.. code-block::

   (method) winch:relax()

----

winch.release_length
====================

 desc

.. code-block::

   (method) winch:release_length(param1: number)

----

winch.set_desired_rate
======================

 desc

.. code-block::

   (method) winch:set_desired_rate(param1: number)
