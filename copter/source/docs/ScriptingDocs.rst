
AC_AttitudeControl
==================

.. code-block::

   table

----

AC_AttitudeControl.get_att_error_angle_deg
==========================================

.. code-block::

   (method) AC_AttitudeControl:get_att_error_angle_deg()
     -> number

----

AC_AttitudeControl.get_rpy_srate
================================

.. code-block::

   (method) AC_AttitudeControl:get_rpy_srate()
     -> number
     2. number
     3. number

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

transfer
--------

.. code-block::

   (method) AP_HAL__I2CDevice_ud:transfer(data_str: string, read_length: integer)
     -> string|nil

 Performs an I2C transfer, sending data_str bytes (see string.pack) and
 returning a string of any requested read bytes (see string.unpack)

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

AP_Scripting_SerialAccess_ud
============================

 Serial port access object

available
---------

.. code-block::

   (method) AP_Scripting_SerialAccess_ud:available()
     -> uint32_t_ud

 Returns number of available bytes to read.

begin
-----

.. code-block::

   (method) AP_Scripting_SerialAccess_ud:begin(baud_rate?: number|uint32_t_ud)

 Start serial port with the given baud rate (no effect for device ports)

@\ *param* ``baud_rate`` — baud rate, parameter-derived value used if nil or omitted

configure_parity
----------------

.. code-block::

   (method) AP_Scripting_SerialAccess_ud:configure_parity(parity: integer)

 Set UART parity (no effect for device ports)

@\ *param* ``parity`` — 0=None, 1=Odd, 2=Even

read
----

.. code-block::

   (method) AP_Scripting_SerialAccess_ud:read()
     -> integer

 Reads a single byte from the serial port

@\ *return* — byte, -1 if error or none available

readstring
----------

.. code-block::

   (method) AP_Scripting_SerialAccess_ud:readstring(count: integer)
     -> string|nil

 Reads up to ``count`` bytes and returns the bytes read as a string. No bytes
 may be read, in which case a 0-length string is returned.

@\ *param* ``count`` — maximum number of bytes to read

@\ *return* — bytes actually read, which may be 0-length, or nil on error

set_flow_control
----------------

.. code-block::

   (method) AP_Scripting_SerialAccess_ud:set_flow_control(flow_control_setting: integer|'0'|'1'|'2')

 Set flow control option for serial port (no effect for device ports)

.. code-block::

   flow_control_setting:
       | '0' -- disabled
       | '1' -- enabled
       | '2' -- auto

set_stop_bits
-------------

.. code-block::

   (method) AP_Scripting_SerialAccess_ud:set_stop_bits(stop_bits: integer)

 Set UART stop bits (no effect for device ports)

@\ *param* ``stop_bits`` — 1 or 2

set_unbuffered_writes
---------------------

.. code-block::

   (method) AP_Scripting_SerialAccess_ud:set_unbuffered_writes(on: boolean)

 Set UART use unbuffered writes flag, false by default (no effect for device ports)

write
-----

.. code-block::

   (method) AP_Scripting_SerialAccess_ud:write(value: integer)
     -> uint32_t_ud

 Writes a single byte

@\ *param* ``value`` — byte to write

@\ *return* — 1 if success else 0

writestring
-----------

.. code-block::

   (method) AP_Scripting_SerialAccess_ud:writestring(data: string)
     -> integer

 Writes a string. The number of bytes actually written, i.e. the length of the
 written prefix of the string, is returned. It may be 0 up to the length of
 the string.

@\ *param* ``data`` — string of bytes to write

@\ *return* — number of bytes actually written, which may be 0

----

AP_Vehicle__custom_mode_state_ud
================================

 Custom mode state, allows customisation of mode behavior

allow_entry
-----------

.. code-block::

   (method) AP_Vehicle__custom_mode_state_ud:allow_entry()
     -> boolean

 get allow_entry, if true the vehicle is allowed to enter this custom mode

----

AR_AttitudeControl
==================

.. code-block::

   table

----

AR_AttitudeControl.get_srate
============================

.. code-block::

   (method) AR_AttitudeControl:get_srate()
     -> number
     2. number

----

BattMonitorScript_State
=======================

.. code-block::

   function BattMonitorScript_State()
     -> BattMonitorScript_State_ud

----

BattMonitorScript_State_ud
==========================

 Object that can be passed to a scripting battery monitor backend

capacity_remaining_pct
----------------------

.. code-block::

   (method) BattMonitorScript_State_ud:capacity_remaining_pct(value: integer)

 set the remaining capacity, if not provided the remaining capacity will be calculated from the consumed mah

@\ *param* ``value`` — 0% to 100%

cell_count
----------

.. code-block::

   (method) BattMonitorScript_State_ud:cell_count(value: integer)

 set the number of avalable cells as set with ``cell_voltages``

cell_voltages
-------------

.. code-block::

   (method) BattMonitorScript_State_ud:cell_voltages(index: integer, value: integer)

 set array field

@\ *param* ``index`` — 0 indexed

@\ *param* ``value`` — voltage in millivolts

consumed_mah
------------

.. code-block::

   (method) BattMonitorScript_State_ud:consumed_mah(value: number)

 set consumed milliampere hours, if not provided the comsumed mah will be calculated from the current draw

@\ *param* ``value`` — milliampere hours

consumed_wh
-----------

.. code-block::

   (method) BattMonitorScript_State_ud:consumed_wh(value: number)

 set consumed watt hours, if not provided the comsumed watt hours will be calculated from the consumed mah and voltage

@\ *param* ``value`` — watt hours

current_amps
------------

.. code-block::

   (method) BattMonitorScript_State_ud:current_amps(value: number)

 set current

@\ *param* ``value`` — amps

cycle_count
-----------

.. code-block::

   (method) BattMonitorScript_State_ud:cycle_count(value: integer)

 set cycle_count

healthy
-------

.. code-block::

   (method) BattMonitorScript_State_ud:healthy(value: boolean)

 set battery monitor health

@\ *param* ``value`` — true if battery monitor is healthy

state_of_health_pct
-------------------

.. code-block::

   (method) BattMonitorScript_State_ud:state_of_health_pct(value: integer)

 set state of health, 255 if not available (this is the default)

temperature
-----------

.. code-block::

   (method) BattMonitorScript_State_ud:temperature(value: number)

 set temperature

@\ *param* ``value`` — degrees Celsius

voltage
-------

.. code-block::

   (method) BattMonitorScript_State_ud:voltage(value: number)

 set voltage

@\ *param* ``value`` — volts

----

CAN
===

.. code-block::

   table

----

CAN.get_device
==============

.. code-block::

   (method) CAN:get_device(buffer_len: number|uint32_t_ud)
     -> ScriptingCANBuffer_ud|nil

----

CAN.get_device2
===============

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

CRSFMenu
========

.. code-block::

   function CRSFMenu(size: integer)
     -> CRSFMenu_ud

----

CRSFMenu_ud
===========

 CRSF menu userdata object

add_menu
--------

.. code-block::

   (method) CRSFMenu_ud:add_menu(name: string)
     -> CRSFMenu_ud|nil

 add a CRSF menu to the menu

@\ *param* ``name`` — menu name for the added menu

@\ *return* — the newly created menu

add_parameter
-------------

.. code-block::

   (method) CRSFMenu_ud:add_parameter(data: string)
     -> CRSFParameter_ud|nil

 add a CRSF parameter to the menu

@\ *param* ``data`` — binary encoded parameter

@\ *return* — the newly created parameter

id
--

.. code-block::

   (method) CRSFMenu_ud:id()
     -> integer

 get id of the menu

name
----

.. code-block::

   (method) CRSFMenu_ud:name()
     -> string

 get name of the menu

num_params
----------

.. code-block::

   (method) CRSFMenu_ud:num_params()
     -> integer

 get the number of parameters in the menu

----

CRSFParameter
=============

.. code-block::

   function CRSFParameter()
     -> CRSFParameter_ud

----

CRSFParameter_ud
================

 CRSF menu parameter userdata object

data
----

.. code-block::

   (method) CRSFParameter_ud:data()
     -> string

 get contents of the parameter as a packed string

id
--

.. code-block::

   (method) CRSFParameter_ud:id()
     -> integer

 get id of the parameter

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

DroneCAN_Handle
===============

.. code-block::

   function DroneCAN_Handle(driver_index: number, signature: uint64_t_ud, data_type: number, canfd?: boolean)
     -> DroneCAN_Handle_ud

----

DroneCAN_Handle_ud
==================

 handle for DroneCAN message operations

broadcast
---------

.. code-block::

   (method) DroneCAN_Handle_ud:broadcast(payload: string)
     -> boolean

 send a DroneCAN broadcast

@\ *param* ``payload`` — payload for message

@\ *return* — true if send succeeded

check_message
-------------

.. code-block::

   (method) DroneCAN_Handle_ud:check_message()
     -> payload: string
     2. nodeid: number
     3. timestamp: uint64_t_ud
     4. canfd: boolean

 check if a new message has arrived for a request or subscription

@\ *return* ``payload`` — payload of the message

@\ *return* ``nodeid`` — node ID the message came from

@\ *return* ``timestamp`` — microseconds since 1/1/1970

@\ *return* ``canfd`` — true if message was CANFD

request
-------

.. code-block::

   (method) DroneCAN_Handle_ud:request(target_node: number, payload: string)
     -> boolean

 make a DroneCAN request

@\ *param* ``target_node`` — node to send request to

@\ *param* ``payload`` — payload for message

@\ *return* — true if send succeeded

subscribe
---------

.. code-block::

   (method) DroneCAN_Handle_ud:subscribe()
     -> boolean

 subscribe to the current signature and data_type

----

DroneCAN_get_FlexDebug
======================

.. code-block::

   function DroneCAN_get_FlexDebug(bus: number, node: number, id: number, last_us: number|uint32_t_ud)
     -> uint32_t_ud|nil
     2. string|nil

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

power_percentage
----------------

.. code-block::

   (method) ESCTelemetryData_ud:power_percentage(pct: integer)

 set power percentage

@\ *param* ``pct`` — range of 0 to 255

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

.. code-block::

   table

----

FWVersion.hash
==============

.. code-block::

   (method) FWVersion:hash()
     -> string

----

FWVersion.major
===============

.. code-block::

   (method) FWVersion:major()
     -> integer

----

FWVersion.minor
===============

.. code-block::

   (method) FWVersion:minor()
     -> integer

----

FWVersion.patch
===============

.. code-block::

   (method) FWVersion:patch()
     -> integer

----

FWVersion.string
================

.. code-block::

   (method) FWVersion:string()
     -> string

----

FWVersion.type
==============

.. code-block::

   (method) FWVersion:type()
     -> integer|'1'|'12'|'13'|'2'...(+4)

----

LED
===

.. code-block::

   table

----

LED.get_rgb
===========

.. code-block::

   (method) LED:get_rgb()
     -> integer
     2. integer
     3. integer

----

Location
========

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

 Deprecated method returning offset from EKF origin

@\ *return* — Vector between origin and location north east up in centimetres

get_vector_from_origin_NEU_cm
-----------------------------

.. code-block::

   (method) Location_ud:get_vector_from_origin_NEU_cm()
     -> Vector3f_ud|nil

 Returns the offset from the EKF origin to this location (in cm)
 Returns nil if the EKF origin wasn’t available at the time this was called.

@\ *return* — Vector between origin and location north east up in cm

get_vector_from_origin_NEU_m
----------------------------

.. code-block::

   (method) Location_ud:get_vector_from_origin_NEU_m()
     -> Vector3f_ud|nil

 Returns the offset from the EKF origin to this location (in metres).
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

set_alt_m
---------

.. code-block::

   (method) Location_ud:set_alt_m(alt: number, frame: integer|'0'|'1'|'2'|'3')

 set altitude in Location object in metres

@\ *param* ``alt`` — altitude

@\ *param* ``frame`` — altitude frame

.. code-block::

   frame:
       | '0' -- ABSOLUTE
       | '1' -- ABOVE_HOME
       | '2' -- ABOVE_ORIGIN
       | '3' -- ABOVE_TERRAIN

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

.. code-block::

   table

----

MotorsMatrix.add_motor_raw
==========================

.. code-block::

   (method) MotorsMatrix:add_motor_raw(motor_num: integer, roll_factor: number, pitch_factor: number, yaw_factor: number, testing_order: integer)

----

MotorsMatrix.get_lost_motor
===========================

.. code-block::

   (method) MotorsMatrix:get_lost_motor()
     -> integer

----

MotorsMatrix.get_thrust_boost
=============================

.. code-block::

   (method) MotorsMatrix:get_thrust_boost()
     -> boolean

----

MotorsMatrix.init
=================

.. code-block::

   (method) MotorsMatrix:init(expected_num_motors: integer)
     -> boolean

----

MotorsMatrix.set_throttle_factor
================================

.. code-block::

   (method) MotorsMatrix:set_throttle_factor(motor_num: integer, throttle_factor: number)
     -> boolean

----

Motors_6DoF
===========

.. code-block::

   table

----

Motors_6DoF.add_motor
=====================

.. code-block::

   (method) Motors_6DoF:add_motor(motor_num: integer, roll_factor: number, pitch_factor: number, yaw_factor: number, throttle_factor: number, forward_factor: number, right_factor: number, reversible: boolean, testing_order: integer)

----

Motors_6DoF.init
================

.. code-block::

   (method) Motors_6DoF:init(expected_num_motors: integer)
     -> boolean

----

Motors_dynamic
==============

.. code-block::

   table

----

Motors_dynamic.add_motor
========================

.. code-block::

   (method) Motors_dynamic:add_motor(motor_num: integer, testing_order: integer)

----

Motors_dynamic.init
===================

.. code-block::

   (method) Motors_dynamic:init(expected_num_motors: integer)
     -> boolean

----

Motors_dynamic.load_factors
===========================

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

 Override RC channel value.  Be wary using this override as it effectively disables RC failsafes

----

RPM
===

.. code-block::

   table

----

RPM.get_rpm
===========

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

.. code-block::

   table

----

SRV_Channels.find_channel
=========================

.. code-block::

   (method) SRV_Channels:find_channel(function_num: integer)
     -> integer|nil

----

SRV_Channels.get_emergency_stop
===============================

.. code-block::

   (method) SRV_Channels:get_emergency_stop()
     -> boolean

----

SRV_Channels.get_output_pwm
===========================

.. code-block::

   (method) SRV_Channels:get_output_pwm(function_num: integer)
     -> integer|nil

----

SRV_Channels.get_output_pwm_chan
================================

.. code-block::

   (method) SRV_Channels:get_output_pwm_chan(chan: integer)
     -> integer|nil

----

SRV_Channels.get_output_scaled
==============================

.. code-block::

   (method) SRV_Channels:get_output_scaled(function_num: integer)
     -> number

----

SRV_Channels.get_safety_state
=============================

.. code-block::

   (method) SRV_Channels:get_safety_state()
     -> boolean

----

SRV_Channels.set_angle
======================

.. code-block::

   (method) SRV_Channels:set_angle(function_num: integer, angle: integer)

----

SRV_Channels.set_output_norm
============================

.. code-block::

   (method) SRV_Channels:set_output_norm(function_num: integer, value: number)

----

SRV_Channels.set_output_pwm
===========================

.. code-block::

   (method) SRV_Channels:set_output_pwm(function_num: integer, pwm: integer)

----

SRV_Channels.set_output_pwm_chan
================================

.. code-block::

   (method) SRV_Channels:set_output_pwm_chan(chan: integer, pwm: integer)

----

SRV_Channels.set_output_pwm_chan_timeout
========================================

.. code-block::

   (method) SRV_Channels:set_output_pwm_chan_timeout(chan: integer, pwm: integer, timeout_ms: integer)

----

SRV_Channels.set_output_scaled
==============================

.. code-block::

   (method) SRV_Channels:set_output_scaled(function_num: integer, value: number)

----

SRV_Channels.set_range
======================

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

ServoTelemetryData
==================

.. code-block::

   function ServoTelemetryData()
     -> ServoTelemetryData_ud

----

ServoTelemetryData_ud
=====================

 Servo telemetry userdata object

command_position
----------------

.. code-block::

   (method) ServoTelemetryData_ud:command_position()
     -> number|nil

 get commanded position

@\ *return* — comanded position in degrees or nil if not available

current
-------

.. code-block::

   (method) ServoTelemetryData_ud:current()
     -> number|nil

 get current

@\ *return* — current in amps or nil if not available

duty_cycle
----------

.. code-block::

   (method) ServoTelemetryData_ud:duty_cycle()
     -> integer|nil

 Get duty cycle

@\ *return* — duty cycle 0% to 100% or nil if not available

force
-----

.. code-block::

   (method) ServoTelemetryData_ud:force()
     -> number|nil

 get force

@\ *return* — force in newton meters or nil if not available

last_update_ms
--------------

.. code-block::

   (method) ServoTelemetryData_ud:last_update_ms()
     -> uint32_t_ud

 Get timestamp of last telem update

@\ *return* — milliseconds since boot

measured_position
-----------------

.. code-block::

   (method) ServoTelemetryData_ud:measured_position()
     -> number|nil

 get measured position

@\ *return* — measured position in degrees or nil if not available

motor_temperature_cdeg
----------------------

.. code-block::

   (method) ServoTelemetryData_ud:motor_temperature_cdeg()
     -> integer|nil

 Get motor temperature in centidegrees

@\ *return* — temperature in centidegrees or nil if not available

pcb_temperature_cdeg
--------------------

.. code-block::

   (method) ServoTelemetryData_ud:pcb_temperature_cdeg()
     -> integer|nil

 Get pcb temperature in centidegrees

@\ *return* — temperature in centidegrees or nil if not available

speed
-----

.. code-block::

   (method) ServoTelemetryData_ud:speed()
     -> number|nil

 get speed

@\ *return* — speed in degrees per second or nil if not available

status_flags
------------

.. code-block::

   (method) ServoTelemetryData_ud:status_flags()
     -> integer|nil

 Get type spesfic status flags

@\ *return* — flags or nil if not available

voltage
-------

.. code-block::

   (method) ServoTelemetryData_ud:voltage()
     -> number|nil

 get voltage

@\ *return* — voltage in volts or nil if not available

----

Socket
======

.. code-block::

   function Socket(datagram: integer)
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

is_pending
----------

.. code-block::

   (method) SocketAPM_ud:is_pending()
     -> boolean

 return true if a socket is in a pending connect state
 used for non-blocking TCP connections

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
     2. uint32_t_ud|nil
     3. integer|nil

 receive data from a socket

@\ *return*

@\ *return* — source IP

@\ *return* — source port

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

sendto
------

.. code-block::

   (method) SocketAPM_ud:sendto(str: string, len: number|uint32_t_ud, ipaddr: number|uint32_t_ud, port: integer)
     -> integer

 send a lua string to a specified address. May contain binary data

@\ *param* ``ipaddr`` — ipv4 address

@\ *param* ``port`` — ipv4 port

set_blocking
------------

.. code-block::

   (method) SocketAPM_ud:set_blocking(blocking: boolean)
     -> boolean

 set blocking state of socket

----

Vector2f
========

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

.. code-block::

   table

----

ahrs.airspeed_EAS
=================

.. code-block::

   (method) ahrs:airspeed_EAS()
     -> number|nil

----

ahrs.airspeed_estimate
======================

.. code-block::

   (method) ahrs:airspeed_estimate()
     -> number|nil

----

ahrs.body_to_earth
==================

.. code-block::

   (method) ahrs:body_to_earth(vector: Vector3f_ud)
     -> Vector3f_ud

----

ahrs.earth_to_body
==================

.. code-block::

   (method) ahrs:earth_to_body(vector: Vector3f_ud)
     -> Vector3f_ud

----

ahrs.get_EAS2TAS
================

.. code-block::

   (method) ahrs:get_EAS2TAS()
     -> number

----

ahrs.get_accel
==============

.. code-block::

   (method) ahrs:get_accel()
     -> Vector3f_ud

----

ahrs.get_gyro
=============

.. code-block::

   (method) ahrs:get_gyro()
     -> Vector3f_ud

----

ahrs.get_hagl
=============

.. code-block::

   (method) ahrs:get_hagl()
     -> number|nil

----

ahrs.get_home
=============

.. code-block::

   (method) ahrs:get_home()
     -> Location_ud

----

ahrs.get_location
=================

.. code-block::

   (method) ahrs:get_location()
     -> Location_ud|nil

----

ahrs.get_origin
===============

.. code-block::

   (method) ahrs:get_origin()
     -> Location_ud|nil

----

ahrs.get_pitch
==============

.. code-block::

   (method) ahrs:get_pitch()
     -> number

----

ahrs.get_pitch_rad
==================

.. code-block::

   (method) ahrs:get_pitch_rad()
     -> number

----

ahrs.get_position
=================

.. code-block::

   (method) ahrs:get_position()
     -> Location_ud|nil

----

ahrs.get_posvelyaw_source_set
=============================

.. code-block::

   (method) ahrs:get_posvelyaw_source_set()
     -> integer

----

ahrs.get_quaternion
===================

.. code-block::

   (method) ahrs:get_quaternion()
     -> Quaternion_ud|nil

----

ahrs.get_relative_position_D_home
=================================

.. code-block::

   (method) ahrs:get_relative_position_D_home()
     -> number

----

ahrs.get_relative_position_NED_home
===================================

.. code-block::

   (method) ahrs:get_relative_position_NED_home()
     -> Vector3f_ud|nil

----

ahrs.get_relative_position_NED_origin
=====================================

.. code-block::

   (method) ahrs:get_relative_position_NED_origin()
     -> Vector3f_ud|nil

----

ahrs.get_roll
=============

.. code-block::

   (method) ahrs:get_roll()
     -> number

----

ahrs.get_roll_rad
=================

.. code-block::

   (method) ahrs:get_roll_rad()
     -> number

----

ahrs.get_variances
==================

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

.. code-block::

   (method) ahrs:get_vel_innovations_and_variances_for_source(source: integer)
     -> Vector3f_ud|nil
     2. Vector3f_ud|nil

----

ahrs.get_velocity_NED
=====================

.. code-block::

   (method) ahrs:get_velocity_NED()
     -> Vector3f_ud|nil

----

ahrs.get_vibration
==================

.. code-block::

   (method) ahrs:get_vibration()
     -> Vector3f_ud

----

ahrs.get_yaw
============

.. code-block::

   (method) ahrs:get_yaw()
     -> number

----

ahrs.get_yaw_rad
================

.. code-block::

   (method) ahrs:get_yaw_rad()
     -> number

----

ahrs.groundspeed_vector
=======================

.. code-block::

   (method) ahrs:groundspeed_vector()
     -> Vector2f_ud

----

ahrs.handle_external_position_estimate
======================================

.. code-block::

   (method) ahrs:handle_external_position_estimate(location: Location_ud, accuracy: number, timestamp_ms: number|uint32_t_ud)
     -> boolean

----

ahrs.head_wind
==============

.. code-block::

   (method) ahrs:head_wind()
     -> number

----

ahrs.healthy
============

.. code-block::

   (method) ahrs:healthy()
     -> boolean

----

ahrs.home_is_set
================

.. code-block::

   (method) ahrs:home_is_set()
     -> boolean

----

ahrs.initialised
================

.. code-block::

   (method) ahrs:initialised()
     -> boolean

----

ahrs.set_home
=============

.. code-block::

   (method) ahrs:set_home(loc: Location_ud)
     -> boolean

----

ahrs.set_origin
===============

.. code-block::

   (method) ahrs:set_origin(loc: Location_ud)
     -> boolean

----

ahrs.set_posvelyaw_source_set
=============================

.. code-block::

   (method) ahrs:set_posvelyaw_source_set(source_set_idx: integer|'0'|'1'|'2')

----

ahrs.wind_alignment
===================

.. code-block::

   (method) ahrs:wind_alignment(heading_deg: number)
     -> number

----

ahrs.wind_estimate
==================

.. code-block::

   (method) ahrs:wind_estimate()
     -> Vector3f_ud

----

analog
======

.. code-block::

   table

----

analog.channel
==============

.. code-block::

   (method) analog:channel()
     -> AP_HAL__AnalogSource_ud|nil

----

analog.mcu_temperature
======================

.. code-block::

   (method) analog:mcu_temperature()
     -> number

----

analog.mcu_voltage
==================

.. code-block::

   (method) analog:mcu_voltage()
     -> number

----

arming
======

.. code-block::

   table

----

arming.arm
==========

.. code-block::

   (method) arming:arm()
     -> boolean

----

arming.arm_force
================

.. code-block::

   (method) arming:arm_force()
     -> boolean

----

arming.disarm
=============

.. code-block::

   (method) arming:disarm()
     -> boolean

----

arming.get_aux_auth_id
======================

.. code-block::

   (method) arming:get_aux_auth_id()
     -> integer|nil

----

arming.is_armed
===============

.. code-block::

   (method) arming:is_armed()
     -> boolean

----

arming.pre_arm_checks
=====================

.. code-block::

   (method) arming:pre_arm_checks()
     -> boolean

----

arming.set_aux_auth_failed
==========================

.. code-block::

   (method) arming:set_aux_auth_failed(auth_id: integer, fail_msg: string)

----

arming.set_aux_auth_passed
==========================

.. code-block::

   (method) arming:set_aux_auth_passed(auth_id: integer)

----

attitude_control
================

.. code-block::

   table

----

attitude_control.set_forward_enable
===================================

.. code-block::

   (method) attitude_control:set_forward_enable(bool: boolean)

----

attitude_control.set_lateral_enable
===================================

.. code-block::

   (method) attitude_control:set_lateral_enable(bool: boolean)

----

attitude_control.set_offset_roll_pitch
======================================

.. code-block::

   (method) attitude_control:set_offset_roll_pitch(roll_deg: number, pitch_deg: number)

----

baro
====

.. code-block::

   table

----

baro.get_altitude
=================

.. code-block::

   (method) baro:get_altitude()
     -> number

----

baro.get_altitude_difference
============================

.. code-block::

   (method) baro:get_altitude_difference(base_pressure: number, pressure: number)
     -> number

----

baro.get_external_temperature
=============================

.. code-block::

   (method) baro:get_external_temperature()
     -> number

----

baro.get_pressure
=================

.. code-block::

   (method) baro:get_pressure()
     -> number

----

baro.get_temperature
====================

.. code-block::

   (method) baro:get_temperature()
     -> number

----

baro.healthy
============

.. code-block::

   (method) baro:healthy(instance: integer)
     -> boolean

----

battery
=======

.. code-block::

   table

----

battery.capacity_remaining_pct
==============================

.. code-block::

   (method) battery:capacity_remaining_pct(instance: integer)
     -> integer|nil

----

battery.consumed_mah
====================

.. code-block::

   (method) battery:consumed_mah(instance: integer)
     -> number|nil

----

battery.consumed_wh
===================

.. code-block::

   (method) battery:consumed_wh(instance: integer)
     -> number|nil

----

battery.current_amps
====================

.. code-block::

   (method) battery:current_amps(instance: integer)
     -> number|nil

----

battery.get_cell_voltage
========================

.. code-block::

   (method) battery:get_cell_voltage(instance: integer, cell: integer)
     -> number|nil

----

battery.get_cycle_count
=======================

.. code-block::

   (method) battery:get_cycle_count(instance: integer)
     -> integer|nil

----

battery.get_resistance
======================

.. code-block::

   (method) battery:get_resistance(instance: integer)
     -> number

----

battery.get_temperature
=======================

.. code-block::

   (method) battery:get_temperature(instance: integer)
     -> number|nil

----

battery.handle_scripting
========================

.. code-block::

   (method) battery:handle_scripting(idx: integer, state: BattMonitorScript_State_ud)
     -> boolean

----

battery.has_failsafed
=====================

.. code-block::

   (method) battery:has_failsafed()
     -> boolean

----

battery.healthy
===============

.. code-block::

   (method) battery:healthy(instance: integer)
     -> boolean

----

battery.num_instances
=====================

.. code-block::

   (method) battery:num_instances()
     -> integer

----

battery.overpower_detected
==========================

.. code-block::

   (method) battery:overpower_detected(instance: integer)
     -> boolean

----

battery.pack_capacity_mah
=========================

.. code-block::

   (method) battery:pack_capacity_mah(instance: integer)
     -> integer

----

battery.reset_remaining
=======================

.. code-block::

   (method) battery:reset_remaining(instance: integer, percentage: number)
     -> boolean

----

battery.voltage
===============

.. code-block::

   (method) battery:voltage(instance: integer)
     -> number

----

battery.voltage_resting_estimate
================================

.. code-block::

   (method) battery:voltage_resting_estimate(instance: integer)
     -> number

----

button
======

.. code-block::

   table

----

button.get_button_state
=======================

.. code-block::

   (method) button:get_button_state(button_number: integer)
     -> boolean

----

camera
======

.. code-block::

   table

----

camera.change_setting
=====================

.. code-block::

   (method) camera:change_setting(instance: integer, setting: integer|'0'|'1'|'2', value: number)
     -> boolean

----

camera.get_state
================

.. code-block::

   (method) camera:get_state(instance: integer)
     -> AP_Camera__camera_state_t_ud|nil

----

camera.record_video
===================

.. code-block::

   (method) camera:record_video(instance: integer, start_recording: boolean)
     -> boolean

----

camera.set_camera_information
=============================

.. code-block::

   (method) camera:set_camera_information(instance: integer, cam_info: mavlink_camera_information_t_ud)

----

camera.set_stream_information
=============================

.. code-block::

   (method) camera:set_stream_information(instance: integer, stream_info: mavlink_video_stream_information_t_ud)

----

camera.set_trigger_distance
===========================

.. code-block::

   (method) camera:set_trigger_distance(instance: integer, distance_m: number)

----

camera.take_picture
===================

.. code-block::

   (method) camera:take_picture(instance: integer)

----

compass
=======

.. code-block::

   table

----

compass.healthy
===============

.. code-block::

   (method) compass:healthy(instance: integer)
     -> boolean

----

crsf
====

.. code-block::

   table

----

crsf.add_menu
=============

.. code-block::

   (method) crsf:add_menu(name: string)
     -> CRSFMenu_ud|nil

----

crsf.get_menu_event
===================

.. code-block::

   (method) crsf:get_menu_event(events: integer|'1'|'2')
     -> integer
     2. string
     3. integer|'1'|'2'

----

crsf.peek_menu_event
====================

.. code-block::

   (method) crsf:peek_menu_event()
     -> integer
     2. integer
     3. string
     4. integer|'1'|'2'

----

crsf.pop_menu_event
===================

.. code-block::

   (method) crsf:pop_menu_event()

----

crsf.send_response
==================

.. code-block::

   (method) crsf:send_response()
     -> boolean

----

crsf.send_write_response
========================

.. code-block::

   (method) crsf:send_write_response(data: string)
     -> boolean

----

dirlist
=======

.. code-block::

   function dirlist(directoryname: string)
     -> table|nil
     2. string|nil

----

efi
===

.. code-block::

   table

----

efi.get_backend
===============

.. code-block::

   (method) efi:get_backend(instance: integer)
     -> AP_EFI_Backend_ud|nil

----

efi.get_last_update_ms
======================

.. code-block::

   (method) efi:get_last_update_ms()
     -> uint32_t_ud

----

efi.get_state
=============

.. code-block::

   (method) efi:get_state()
     -> EFI_State_ud

----

esc_telem
=========

.. code-block::

   table

----

esc_telem.get_consumption_mah
=============================

.. code-block::

   (method) esc_telem:get_consumption_mah(instance: integer)
     -> number|nil

----

esc_telem.get_current
=====================

.. code-block::

   (method) esc_telem:get_current(instance: integer)
     -> number|nil

----

esc_telem.get_last_telem_data_ms
================================

.. code-block::

   (method) esc_telem:get_last_telem_data_ms(esc_index: integer)
     -> uint32_t_ud

----

esc_telem.get_motor_temperature
===============================

.. code-block::

   (method) esc_telem:get_motor_temperature(instance: integer)
     -> integer|nil

----

esc_telem.get_rpm
=================

.. code-block::

   (method) esc_telem:get_rpm(instance: integer)
     -> number|nil

----

esc_telem.get_temperature
=========================

.. code-block::

   (method) esc_telem:get_temperature(instance: integer)
     -> integer|nil

----

esc_telem.get_usage_seconds
===========================

.. code-block::

   (method) esc_telem:get_usage_seconds(instance: integer)
     -> uint32_t_ud|nil

----

esc_telem.get_voltage
=====================

.. code-block::

   (method) esc_telem:get_voltage(instance: integer)
     -> number|nil

----

esc_telem.set_rpm_scale
=======================

.. code-block::

   (method) esc_telem:set_rpm_scale(esc_index: integer, scale_factor: number)

----

esc_telem.update_rpm
====================

.. code-block::

   (method) esc_telem:update_rpm(esc_index: integer, rpm: number, error_rate: number)

----

esc_telem.update_telem_data
===========================

.. code-block::

   (method) esc_telem:update_telem_data(instance: integer, telemdata: ESCTelemetryData_ud, data_mask: integer)

----

fence
=====

.. code-block::

   table

----

fence.get_breach_direction_NED
==============================

.. code-block::

   (method) fence:get_breach_direction_NED(fence_type: integer|1|2|4|8)
     -> Vector3f_ud|nil
     2. Location_ud|nil

----

fence.get_breach_distance
=========================

.. code-block::

   (method) fence:get_breach_distance(fence_type: integer|1|2|4|8)
     -> number

----

fence.get_breach_time
=====================

.. code-block::

   (method) fence:get_breach_time()
     -> system_time: uint32_t_ud

----

fence.get_breaches
==================

.. code-block::

   (method) fence:get_breaches()
     -> integer|1|2|4|8

----

fence.get_enabled_fences
========================

.. code-block::

   (method) fence:get_enabled_fences()
     -> integer|1|2|4|8

----

fence.get_margin_breach_time
============================

.. code-block::

   (method) fence:get_margin_breach_time()
     -> system_time: uint32_t_ud

----

fence.get_margin_breaches
=========================

.. code-block::

   (method) fence:get_margin_breaches()
     -> integer|1|2|4|8

----

fence.get_safe_alt_max
======================

.. code-block::

   (method) fence:get_safe_alt_max()
     -> altitude_m: number
     2. frame: integer

----

fence.get_safe_alt_min
======================

.. code-block::

   (method) fence:get_safe_alt_min()
     -> altitude_m: number
     2. frame: integer

----

fence.present
=============

.. code-block::

   (method) fence:present()
     -> integer|1|2|4|8

----

follow
======

.. code-block::

   table

----

follow.get_last_update_ms
=========================

.. code-block::

   (method) follow:get_last_update_ms()
     -> uint32_t_ud

----

follow.get_target_dist_and_vel_NED_m
====================================

.. code-block::

   (method) follow:get_target_dist_and_vel_NED_m()
     -> Vector3f_ud|nil
     2. Vector3f_ud|nil
     3. Vector3f_ud|nil

----

follow.get_target_heading_deg
=============================

.. code-block::

   (method) follow:get_target_heading_deg()
     -> number|nil

----

follow.get_target_location_and_velocity
=======================================

.. code-block::

   (method) follow:get_target_location_and_velocity()
     -> Location_ud|nil
     2. Vector3f_ud|nil

----

follow.get_target_location_and_velocity_ofs
===========================================

.. code-block::

   (method) follow:get_target_location_and_velocity_ofs()
     -> Location_ud|nil
     2. Vector3f_ud|nil

----

follow.get_target_sysid
=======================

.. code-block::

   (method) follow:get_target_sysid()
     -> uint32_t_ud

----

follow.have_target
==================

.. code-block::

   (method) follow:have_target()
     -> boolean

----

frsky_sport
===========

.. code-block::

   table

----

frsky_sport.prep_number
=======================

.. code-block::

   (method) frsky_sport:prep_number(number: integer, digits: integer, power: integer)
     -> integer

----

frsky_sport.sport_telemetry_push
================================

.. code-block::

   (method) frsky_sport:sport_telemetry_push(sensor: integer, frame: integer, appid: integer, data: integer)
     -> boolean

----

fs
==

.. code-block::

   table

----

fs.crc32
========

.. code-block::

   (method) fs:crc32(file_name: string)
     -> uint32_t_ud|nil

----

fs.format
=========

.. code-block::

   (method) fs:format()
     -> boolean

----

fs.get_format_status
====================

.. code-block::

   (method) fs:get_format_status()
     -> integer

----

fs.stat
=======

.. code-block::

   (method) fs:stat(param1: string)
     -> stat_t_ud|nil

----

gcs
===

.. code-block::

   table

----

gcs.enable_high_latency_connections
===================================

.. code-block::

   (method) gcs:enable_high_latency_connections(enabled: boolean)

----

gcs.frame_type
==============

.. code-block::

   (method) gcs:frame_type()
     -> integer|'0'|'1'|'10'|'11'...(+40)

----

gcs.get_allow_param_set
=======================

.. code-block::

   (method) gcs:get_allow_param_set()
     -> boolean

----

gcs.get_high_latency_status
===========================

.. code-block::

   (method) gcs:get_high_latency_status()
     -> boolean

----

gcs.get_hud_throttle
====================

.. code-block::

   (method) gcs:get_hud_throttle()
     -> integer

----

gcs.last_seen
=============

.. code-block::

   (method) gcs:last_seen()
     -> uint32_t_ud

----

gcs.run_command_int
===================

.. code-block::

   (method) gcs:run_command_int(command: integer, params: table)
     -> integer

----

gcs.send_named_float
====================

.. code-block::

   (method) gcs:send_named_float(name: string, value: number)

----

gcs.send_named_int
==================

.. code-block::

   (method) gcs:send_named_int(name: string, value: integer)

----

gcs.send_named_string
=====================

.. code-block::

   (method) gcs:send_named_string(name: string, value: string)

----

gcs.send_text
=============

.. code-block::

   (method) gcs:send_text(severity: integer|'0'|'1'|'2'|'3'...(+4), text: string)

----

gcs.set_allow_param_set
=======================

.. code-block::

   (method) gcs:set_allow_param_set(new_allow_value: boolean)

----

gcs.set_message_interval
========================

.. code-block::

   (method) gcs:set_message_interval(port_num: integer, msg_id: number|uint32_t_ud, interval_us: integer)
     -> integer|'0'|'4'

----

gpio
====

.. code-block::

   table

----

gpio.getPinFullMode
===================

.. code-block::

   (method) gpio:getPinFullMode(pin_number: integer)
     -> uint32_t_ud|nil

----

gpio.get_mode
=============

.. code-block::

   (method) gpio:get_mode(pin_number: integer)
     -> uint32_t_ud|nil

----

gpio.pinMode
============

.. code-block::

   (method) gpio:pinMode(pin_number: integer, mode: integer|'0'|'1')

----

gpio.read
=========

.. code-block::

   (method) gpio:read(pin_number: integer)
     -> boolean

----

gpio.setPinFullMode
===================

.. code-block::

   (method) gpio:setPinFullMode(pin_number: integer, mode: number|uint32_t_ud)

----

gpio.set_mode
=============

.. code-block::

   (method) gpio:set_mode(pin_number: integer, mode: number|uint32_t_ud)

----

gpio.toggle
===========

.. code-block::

   (method) gpio:toggle(pin_number: integer)

----

gpio.write
==========

.. code-block::

   (method) gpio:write(pin_number: integer, value: integer|'0'|'1')

----

gps
===

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

.. code-block::

   (method) gps:first_unconfigured_gps()
     -> integer|nil

----

gps.get_antenna_offset
======================

.. code-block::

   (method) gps:get_antenna_offset(instance: integer)
     -> Vector3f_ud

----

gps.get_hdop
============

.. code-block::

   (method) gps:get_hdop(instance: integer)
     -> integer

----

gps.get_vdop
============

.. code-block::

   (method) gps:get_vdop(instance: integer)
     -> integer

----

gps.gps_yaw_deg
===============

.. code-block::

   (method) gps:gps_yaw_deg(instance: integer)
     -> number|nil
     2. number|nil
     3. uint32_t_ud|nil

----

gps.ground_course
=================

.. code-block::

   (method) gps:ground_course(instance: integer)
     -> number

----

gps.ground_speed
================

.. code-block::

   (method) gps:ground_speed(instance: integer)
     -> number

----

gps.have_vertical_velocity
==========================

.. code-block::

   (method) gps:have_vertical_velocity(instance: integer)
     -> boolean

----

gps.horizontal_accuracy
=======================

.. code-block::

   (method) gps:horizontal_accuracy(instance: integer)
     -> number|nil

----

gps.inject_data
===============

.. code-block::

   (method) gps:inject_data(data: string)

----

gps.last_fix_time_ms
====================

.. code-block::

   (method) gps:last_fix_time_ms(instance: integer)
     -> uint32_t_ud

----

gps.last_message_time_ms
========================

.. code-block::

   (method) gps:last_message_time_ms(instance: integer)
     -> uint32_t_ud

----

gps.location
============

.. code-block::

   (method) gps:location(instance: integer)
     -> Location_ud

----

gps.num_sats
============

.. code-block::

   (method) gps:num_sats(instance: integer)
     -> integer

----

gps.num_sensors
===============

.. code-block::

   (method) gps:num_sensors()
     -> integer

----

gps.primary_sensor
==================

.. code-block::

   (method) gps:primary_sensor()
     -> integer

----

gps.speed_accuracy
==================

.. code-block::

   (method) gps:speed_accuracy(instance: integer)
     -> number|nil

----

gps.status
==========

.. code-block::

   (method) gps:status(instance: integer)
     -> integer

----

gps.time_epoch_usec
===================

.. code-block::

   (method) gps:time_epoch_usec(instance: integer)
     -> uint64_t_ud

----

gps.time_week
=============

.. code-block::

   (method) gps:time_week(instance: integer)
     -> integer

----

gps.time_week_ms
================

.. code-block::

   (method) gps:time_week_ms(instance: integer)
     -> uint32_t_ud

----

gps.velocity
============

.. code-block::

   (method) gps:velocity(instance: integer)
     -> Vector3f_ud

----

gps.vertical_accuracy
=====================

.. code-block::

   (method) gps:vertical_accuracy(instance: integer)
     -> number|nil

----

i2c
===

.. code-block::

   table

----

i2c.get_device
==============

.. code-block::

   (method) i2c:get_device(bus: integer, address: integer, clock?: number|uint32_t_ud, smbus?: boolean)
     -> AP_HAL__I2CDevice_ud

----

ins
===

.. code-block::

   table

----

ins.accels_consistent
=====================

.. code-block::

   (method) ins:accels_consistent(threshold: number)
     -> boolean

----

ins.calibrating
===============

.. code-block::

   (method) ins:calibrating()
     -> boolean

----

ins.get_accel
=============

.. code-block::

   (method) ins:get_accel(instance: integer)
     -> Vector3f_ud

----

ins.get_accel_health
====================

.. code-block::

   (method) ins:get_accel_health(instance: integer)
     -> boolean

----

ins.get_gyro
============

.. code-block::

   (method) ins:get_gyro(instance: integer)
     -> Vector3f_ud

----

ins.get_gyro_health
===================

.. code-block::

   (method) ins:get_gyro_health(instance: integer)
     -> boolean

----

ins.get_temperature
===================

.. code-block::

   (method) ins:get_temperature(instance: integer)
     -> number

----

ins.gyros_consistent
====================

.. code-block::

   (method) ins:gyros_consistent(threshold: integer)
     -> boolean

----

iomcu
=====

.. code-block::

   table

----

iomcu.healthy
=============

.. code-block::

   (method) iomcu:healthy()
     -> boolean

----

ipv4_addr_to_string
===================

.. code-block::

   function ipv4_addr_to_string(addr: number|uint32_t_ud)
     -> string

----

logger
======

.. code-block::

   table

----

logger.log_file_content
=======================

.. code-block::

   (method) logger:log_file_content(filename: string)

----

logger.write
============

.. code-block::

   (method) logger:write(name: string, labels: string, format: string, units: string, multipliers: string, ...boolean|string|number|uint32_t_ud)

.. code-block::

   (method) logger:write(name: string, labels: string, format: string, ...boolean|string|number|uint32_t_ud)

----

mavlink
=======

.. code-block::

   table

----

mavlink.block_command
=====================

.. code-block::

   (method) mavlink:block_command(comand_id: integer)
     -> boolean

----

mavlink.init
============

.. code-block::

   (method) mavlink:init(msg_queue_length: number|uint32_t_ud, num_rx_msgid: number|uint32_t_ud)

----

mavlink.receive_chan
====================

.. code-block::

   (method) mavlink:receive_chan()
     -> string
     2. number
     3. uint32_t_ud

----

mavlink.register_rx_msgid
=========================

.. code-block::

   (method) mavlink:register_rx_msgid(msg_id: number)
     -> boolean

----

mavlink.send_chan
=================

.. code-block::

   (method) mavlink:send_chan(chan: integer, msgid: integer, message: string)
     -> boolean|nil

----

mavlink_camera_information_t
============================

.. code-block::

   function mavlink_camera_information_t()
     -> mavlink_camera_information_t_ud

----

mavlink_camera_information_t_ud
===============================

 The MAVLink CAMERA_INFORMATION message struct

cam_definition_uri
------------------

.. code-block::

   (method) mavlink_camera_information_t_ud:cam_definition_uri(index: integer)
     -> integer

 get array field

cam_definition_version
----------------------

.. code-block::

   (method) mavlink_camera_information_t_ud:cam_definition_version()
     -> integer

 get field

firmware_version
----------------

.. code-block::

   (method) mavlink_camera_information_t_ud:firmware_version()
     -> uint32_t_ud

 get field

flags
-----

.. code-block::

   (method) mavlink_camera_information_t_ud:flags()
     -> uint32_t_ud

 get field

focal_length
------------

.. code-block::

   (method) mavlink_camera_information_t_ud:focal_length()
     -> number

 get field

gimbal_device_id
----------------

.. code-block::

   (method) mavlink_camera_information_t_ud:gimbal_device_id()
     -> integer

 get field

lens_id
-------

.. code-block::

   (method) mavlink_camera_information_t_ud:lens_id()
     -> integer

 get field

model_name
----------

.. code-block::

   (method) mavlink_camera_information_t_ud:model_name(index: integer)
     -> integer

 get array field

resolution_h
------------

.. code-block::

   (method) mavlink_camera_information_t_ud:resolution_h()
     -> integer

 get field

resolution_v
------------

.. code-block::

   (method) mavlink_camera_information_t_ud:resolution_v()
     -> integer

 get field

sensor_size_h
-------------

.. code-block::

   (method) mavlink_camera_information_t_ud:sensor_size_h()
     -> number

 get field

sensor_size_v
-------------

.. code-block::

   (method) mavlink_camera_information_t_ud:sensor_size_v()
     -> number

 get field

time_boot_ms
------------

.. code-block::

   (method) mavlink_camera_information_t_ud:time_boot_ms()
     -> uint32_t_ud

 get field

vendor_name
-----------

.. code-block::

   (method) mavlink_camera_information_t_ud:vendor_name(index: integer)
     -> integer

 get array field

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

mavlink_video_stream_information_t
==================================

.. code-block::

   function mavlink_video_stream_information_t()
     -> mavlink_video_stream_information_t_ud

----

mavlink_video_stream_information_t_ud
=====================================

 The MAVLink VIDEO_STREAM_INFORMATION message struct

bitrate
-------

.. code-block::

   (method) mavlink_video_stream_information_t_ud:bitrate()
     -> uint32_t_ud

 get field

count
-----

.. code-block::

   (method) mavlink_video_stream_information_t_ud:count()
     -> integer

 get field

encoding
--------

.. code-block::

   (method) mavlink_video_stream_information_t_ud:encoding()
     -> integer

 get field

flags
-----

.. code-block::

   (method) mavlink_video_stream_information_t_ud:flags()
     -> integer

 get field

framerate
---------

.. code-block::

   (method) mavlink_video_stream_information_t_ud:framerate()
     -> number

 get field

hfov
----

.. code-block::

   (method) mavlink_video_stream_information_t_ud:hfov()
     -> integer

 get field

name
----

.. code-block::

   (method) mavlink_video_stream_information_t_ud:name(index: integer)
     -> integer

 get array field

resolution_h
------------

.. code-block::

   (method) mavlink_video_stream_information_t_ud:resolution_h()
     -> integer

 get field

resolution_v
------------

.. code-block::

   (method) mavlink_video_stream_information_t_ud:resolution_v()
     -> integer

 get field

rotation
--------

.. code-block::

   (method) mavlink_video_stream_information_t_ud:rotation()
     -> integer

 get field

stream_id
---------

.. code-block::

   (method) mavlink_video_stream_information_t_ud:stream_id()
     -> integer

 get field

type
----

.. code-block::

   (method) mavlink_video_stream_information_t_ud:type()
     -> integer

 get field

uri
---

.. code-block::

   (method) mavlink_video_stream_information_t_ud:uri(index: integer)
     -> integer

 get array field

----

micros
======

.. code-block::

   function micros()
     -> uint32_t_ud

----

millis
======

.. code-block::

   function millis()
     -> uint32_t_ud

----

mission
=======

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

.. code-block::

   (method) mission:clear()
     -> boolean

----

mission.cmd_has_location
========================

.. code-block::

   (method) mission:cmd_has_location(cmd: integer)
     -> boolean

----

mission.get_current_do_cmd_id
=============================

.. code-block::

   (method) mission:get_current_do_cmd_id()
     -> integer

----

mission.get_current_nav_id
==========================

.. code-block::

   (method) mission:get_current_nav_id()
     -> integer

----

mission.get_current_nav_index
=============================

.. code-block::

   (method) mission:get_current_nav_index()
     -> integer

----

mission.get_index_of_jump_tag
=============================

.. code-block::

   (method) mission:get_index_of_jump_tag(tag: integer)
     -> integer

----

mission.get_item
================

.. code-block::

   (method) mission:get_item(index: integer)
     -> mavlink_mission_item_int_t_ud|nil

----

mission.get_last_jump_tag
=========================

.. code-block::

   (method) mission:get_last_jump_tag()
     -> integer|nil
     2. integer|nil

----

mission.get_prev_nav_cmd_id
===========================

.. code-block::

   (method) mission:get_prev_nav_cmd_id()
     -> integer

----

mission.jump_to_abort_landing_sequence
======================================

.. code-block::

   (method) mission:jump_to_abort_landing_sequence()
     -> boolean

----

mission.jump_to_landing_sequence
================================

.. code-block::

   (method) mission:jump_to_landing_sequence()
     -> boolean

----

mission.jump_to_tag
===================

.. code-block::

   (method) mission:jump_to_tag(tag: integer)
     -> boolean

----

mission.num_commands
====================

.. code-block::

   (method) mission:num_commands()
     -> integer

----

mission.set_current_cmd
=======================

.. code-block::

   (method) mission:set_current_cmd(index: integer)
     -> boolean

----

mission.set_item
================

.. code-block::

   (method) mission:set_item(index: integer, item: mavlink_mission_item_int_t_ud)
     -> boolean

----

mission.state
=============

.. code-block::

   (method) mission:state()
     -> integer

----

mission_receive
===============

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

.. code-block::

   table

----

motors.get_desired_spool_state
==============================

.. code-block::

   (method) motors:get_desired_spool_state()
     -> integer

----

motors.get_forward
==================

.. code-block::

   (method) motors:get_forward()
     -> number

----

motors.get_interlock
====================

.. code-block::

   (method) motors:get_interlock()
     -> boolean

----

motors.get_lateral
==================

.. code-block::

   (method) motors:get_lateral()
     -> number

----

motors.get_pitch
================

.. code-block::

   (method) motors:get_pitch()
     -> number

----

motors.get_pitch_ff
===================

.. code-block::

   (method) motors:get_pitch_ff()
     -> number

----

motors.get_roll
===============

.. code-block::

   (method) motors:get_roll()
     -> number

----

motors.get_roll_ff
==================

.. code-block::

   (method) motors:get_roll_ff()
     -> number

----

motors.get_spool_state
======================

.. code-block::

   (method) motors:get_spool_state()
     -> integer|'0'|'1'|'2'|'3'...(+1)

----

motors.get_throttle
===================

.. code-block::

   (method) motors:get_throttle()
     -> number

----

motors.get_throttle_in
======================

.. code-block::

   (method) motors:get_throttle_in()
     -> number

----

motors.get_yaw
==============

.. code-block::

   (method) motors:get_yaw()
     -> number

----

motors.get_yaw_ff
=================

.. code-block::

   (method) motors:get_yaw_ff()
     -> number

----

motors.set_external_limits
==========================

.. code-block::

   (method) motors:set_external_limits(roll: boolean, pitch: boolean, yaw: boolean, throttle_lower: boolean, throttle_upper: boolean)

----

motors.set_frame_string
=======================

.. code-block::

   (method) motors:set_frame_string(param1: string)

----

mount
=====

.. code-block::

   table

----

mount.get_angle_target
======================

.. code-block::

   (method) mount:get_angle_target(instance: integer)
     -> number|nil
     2. number|nil
     3. number|nil
     4. boolean|nil

----

mount.get_attitude_euler
========================

.. code-block::

   (method) mount:get_attitude_euler(instance: integer)
     -> number|nil
     2. number|nil
     3. number|nil

----

mount.get_location_target
=========================

.. code-block::

   (method) mount:get_location_target(instance: integer)
     -> Location_ud|nil

----

mount.get_mode
==============

.. code-block::

   (method) mount:get_mode(instance: integer)
     -> integer|'0'|'1'|'2'|'3'...(+3)

----

mount.get_rate_target
=====================

.. code-block::

   (method) mount:get_rate_target(instance: integer)
     -> number|nil
     2. number|nil
     3. number|nil
     4. boolean|nil

----

mount.set_angle_target
======================

.. code-block::

   (method) mount:set_angle_target(instance: integer, roll_deg: number, pitch_deg: number, yaw_deg: number, yaw_is_earth_frame: boolean)

----

mount.set_attitude_euler
========================

.. code-block::

   (method) mount:set_attitude_euler(instance: integer, roll_deg: number, pitch_deg: number, yaw_deg: number)

----

mount.set_mode
==============

.. code-block::

   (method) mount:set_mode(instance: integer, mode: integer|'0'|'1'|'2'|'3'...(+3))

----

mount.set_natively_supported_mount_target_types
===============================================

.. code-block::

   (method) mount:set_natively_supported_mount_target_types(instance: integer, types_mask: integer|'1'|'16'|'2'|'4'...(+1))

----

mount.set_rate_target
=====================

.. code-block::

   (method) mount:set_rate_target(instance: integer, roll_degs: number, pitch_degs: number, yaw_degs: number, yaw_is_earth_frame: boolean)

----

mount.set_roi_target
====================

.. code-block::

   (method) mount:set_roi_target(instance: integer, target_loc: Location_ud)

----

networking
==========

.. code-block::

   table

----

networking.add_route
====================

.. code-block::

   (method) networking:add_route(backend_idx: integer, iface_idx: integer, dest_ip: number|uint32_t_ud, mask_len: integer)
     -> boolean

----

networking.address_to_str
=========================

.. code-block::

   (method) networking:address_to_str(ip4addr: number|uint32_t_ud)
     -> string

----

networking.get_gateway_active
=============================

.. code-block::

   (method) networking:get_gateway_active()
     -> uint32_t_ud

----

networking.get_ip_active
========================

.. code-block::

   (method) networking:get_ip_active()
     -> uint32_t_ud

----

networking.get_netmask_active
=============================

.. code-block::

   (method) networking:get_netmask_active()
     -> uint32_t_ud

----

notify
======

.. code-block::

   table

----

notify.handle_rgb
=================

.. code-block::

   (method) notify:handle_rgb(red: integer, green: integer, blue: integer, rate_hz: integer)

----

notify.handle_rgb_id
====================

.. code-block::

   (method) notify:handle_rgb_id(red: integer, green: integer, blue: integer, id: integer)

----

notify.play_tune
================

.. code-block::

   (method) notify:play_tune(tune: string)

----

notify.release_text
===================

.. code-block::

   (method) notify:release_text(row: integer)

----

notify.send_text
================

.. code-block::

   (method) notify:send_text(text: string, row: integer)

----

onvif
=====

.. code-block::

   table

----

onvif.get_pan_tilt_limit_max
============================

.. code-block::

   (method) onvif:get_pan_tilt_limit_max()
     -> Vector2f_ud

----

onvif.get_pan_tilt_limit_min
============================

.. code-block::

   (method) onvif:get_pan_tilt_limit_min()
     -> Vector2f_ud

----

onvif.set_absolutemove
======================

.. code-block::

   (method) onvif:set_absolutemove(pan: number, tilt: number, zoom: number)
     -> boolean

----

onvif.start
===========

.. code-block::

   (method) onvif:start(username: string, password: string, httphostname: string)
     -> boolean

----

optical_flow
============

.. code-block::

   table

----

optical_flow.enabled
====================

.. code-block::

   (method) optical_flow:enabled()
     -> boolean

----

optical_flow.healthy
====================

.. code-block::

   (method) optical_flow:healthy()
     -> boolean

----

optical_flow.quality
====================

.. code-block::

   (method) optical_flow:quality()
     -> integer

----

param
=====

.. code-block::

   table

----

param.add_param
===============

.. code-block::

   (method) param:add_param(table_key: integer, param_num: integer, name: string, default_value: number)
     -> boolean

----

param.add_table
===============

.. code-block::

   (method) param:add_table(table_key: integer, prefix: string, num_params: integer)
     -> boolean

----

param.get
=========

.. code-block::

   (method) param:get(name: string)
     -> number|nil

----

param.set
=========

.. code-block::

   (method) param:set(name: string, value: number)
     -> boolean

----

param.set_and_save
==================

.. code-block::

   (method) param:set_and_save(name: string, value: number)
     -> boolean

----

param.set_default
=================

.. code-block::

   (method) param:set_default(name: string, value: number)
     -> boolean

----

periph
======

.. code-block::

   table

----

periph.can_printf
=================

.. code-block::

   (method) periph:can_printf(text: string)

----

periph.get_vehicle_state
========================

.. code-block::

   (method) periph:get_vehicle_state()
     -> uint64_t_ud

----

periph.get_yaw_earth
====================

.. code-block::

   (method) periph:get_yaw_earth()
     -> number

----

periph.reboot
=============

.. code-block::

   (method) periph:reboot(hold_in_bootloader: boolean)

----

poscontrol
==========

.. code-block::

   table

----

poscontrol.get_accel_target
===========================

.. code-block::

   (method) poscontrol:get_accel_target()
     -> Vector3f_ud|nil

----

poscontrol.get_posvelaccel_offset
=================================

.. code-block::

   (method) poscontrol:get_posvelaccel_offset()
     -> Vector3f_ud|nil
     2. Vector3f_ud|nil
     3. Vector3f_ud|nil

----

poscontrol.get_vel_target
=========================

.. code-block::

   (method) poscontrol:get_vel_target()
     -> Vector3f_ud|nil

----

poscontrol.set_posvelaccel_offset
=================================

.. code-block::

   (method) poscontrol:set_posvelaccel_offset(pos_offset_NED: Vector3f_ud, vel_offset_NED: Vector3f_ud, accel_offset_NED: Vector3f_ud)
     -> boolean

----

precland
========

.. code-block::

   table

----

precland.get_last_valid_target_ms
=================================

.. code-block::

   (method) precland:get_last_valid_target_ms()
     -> uint32_t_ud

----

precland.get_target_location
============================

.. code-block::

   (method) precland:get_target_location()
     -> Location_ud|nil

----

precland.get_target_velocity
============================

.. code-block::

   (method) precland:get_target_velocity()
     -> Vector2f_ud|nil

----

precland.healthy
================

.. code-block::

   (method) precland:healthy()
     -> boolean

----

precland.target_acquired
========================

.. code-block::

   (method) precland:target_acquired()
     -> boolean

----

print
=====

.. code-block::

   function print(text: string|number)

----

proximity
=========

.. code-block::

   table

----

proximity.get_backend
=====================

.. code-block::

   (method) proximity:get_backend(instance: integer)
     -> AP_Proximity_Backend_ud|nil

----

proximity.get_closest_object
============================

.. code-block::

   (method) proximity:get_closest_object()
     -> number|nil
     2. number|nil

----

proximity.get_object_angle_and_distance
=======================================

.. code-block::

   (method) proximity:get_object_angle_and_distance(object_number: integer)
     -> number|nil
     2. number|nil

----

proximity.get_object_count
==========================

.. code-block::

   (method) proximity:get_object_count()
     -> integer

----

proximity.get_status
====================

.. code-block::

   (method) proximity:get_status()
     -> integer

----

proximity.num_sensors
=====================

.. code-block::

   (method) proximity:num_sensors()
     -> integer

----

quadplane
=========

.. code-block::

   table

----

quadplane.abort_landing
=======================

.. code-block::

   (method) quadplane:abort_landing()
     -> boolean

----

quadplane.in_assisted_flight
============================

.. code-block::

   (method) quadplane:in_assisted_flight()
     -> boolean

----

quadplane.in_vtol_land_descent
==============================

.. code-block::

   (method) quadplane:in_vtol_land_descent()
     -> boolean

----

quadplane.in_vtol_mode
======================

.. code-block::

   (method) quadplane:in_vtol_mode()
     -> boolean

----

rally
=====

.. code-block::

   table

----

rally.get_rally_location
========================

.. code-block::

   (method) rally:get_rally_location(index: integer)
     -> Location_ud|nil

----

rangefinder
===========

.. code-block::

   table

----

rangefinder.distance_cm_orient
==============================

.. code-block::

   (method) rangefinder:distance_cm_orient(orientation: integer)
     -> integer

----

rangefinder.distance_orient
===========================

.. code-block::

   (method) rangefinder:distance_orient(orientation: integer)
     -> number

----

rangefinder.get_backend
=======================

.. code-block::

   (method) rangefinder:get_backend(rangefinder_instance: integer)
     -> AP_RangeFinder_Backend_ud|nil

----

rangefinder.get_pos_offset_orient
=================================

.. code-block::

   (method) rangefinder:get_pos_offset_orient(orientation: integer)
     -> Vector3f_ud

----

rangefinder.ground_clearance_cm_orient
======================================

.. code-block::

   (method) rangefinder:ground_clearance_cm_orient(orientation: integer)
     -> integer

----

rangefinder.ground_clearance_orient
===================================

.. code-block::

   (method) rangefinder:ground_clearance_orient(orientation: integer)
     -> number

----

rangefinder.has_data_orient
===========================

.. code-block::

   (method) rangefinder:has_data_orient(orientation: integer)
     -> boolean

----

rangefinder.has_orientation
===========================

.. code-block::

   (method) rangefinder:has_orientation(orientation: integer)
     -> boolean

----

rangefinder.max_distance_cm_orient
==================================

.. code-block::

   (method) rangefinder:max_distance_cm_orient(orientation: integer)
     -> integer

----

rangefinder.max_distance_orient
===============================

.. code-block::

   (method) rangefinder:max_distance_orient(orientation: integer)
     -> number

----

rangefinder.min_distance_cm_orient
==================================

.. code-block::

   (method) rangefinder:min_distance_cm_orient(orientation: integer)
     -> integer

----

rangefinder.min_distance_orient
===============================

.. code-block::

   (method) rangefinder:min_distance_orient(orientation: integer)
     -> number

----

rangefinder.num_sensors
=======================

.. code-block::

   (method) rangefinder:num_sensors()
     -> integer

----

rangefinder.signal_quality_pct_orient
=====================================

.. code-block::

   (method) rangefinder:signal_quality_pct_orient(orientation: integer)
     -> integer

----

rangefinder.status_orient
=========================

.. code-block::

   (method) rangefinder:status_orient(orientation: integer)
     -> integer

----

rc
==

.. code-block::

   table

----

rc.find_channel_for_option
==========================

.. code-block::

   (method) rc:find_channel_for_option(aux_fun: integer)
     -> RC_Channel_ud|nil

----

rc.get_aux_cached
=================

.. code-block::

   (method) rc:get_aux_cached(aux_fn: integer)
     -> integer|nil

----

rc.get_channel
==============

.. code-block::

   (method) rc:get_channel(chan_num: integer)
     -> RC_Channel_ud|nil

----

rc.get_pwm
==========

.. code-block::

   (method) rc:get_pwm(chan_num: integer)
     -> integer|nil

----

rc.has_valid_input
==================

.. code-block::

   (method) rc:has_valid_input()
     -> boolean

----

rc.run_aux_function
===================

.. code-block::

   (method) rc:run_aux_function(aux_fun: integer, ch_flag: integer|'0'|'1'|'2')
     -> boolean

----

relay
=====

.. code-block::

   table

----

relay.enabled
=============

.. code-block::

   (method) relay:enabled(instance: integer)
     -> boolean

----

relay.get
=========

.. code-block::

   (method) relay:get(instance: integer)
     -> integer

----

relay.off
=========

.. code-block::

   (method) relay:off(instance: integer)

----

relay.on
========

.. code-block::

   (method) relay:on(instance: integer)

----

relay.toggle
============

.. code-block::

   (method) relay:toggle(instance: integer)

----

remove
======

.. code-block::

   function remove(filename: string)
     -> boolean|nil
     2. string|nil
     3. integer

----

rtc
===

.. code-block::

   table

----

rtc.clock_s_to_date_fields
==========================

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

.. code-block::

   (method) rtc:date_fields_to_clock_s(year: integer, month: integer, day: integer, hour: integer, min: integer, sec: integer)
     -> uint32_t_ud

----

scripting
=========

.. code-block::

   table

----

scripting.restart_all
=====================

.. code-block::

   (method) scripting:restart_all()

----

serial
======

.. code-block::

   table

----

serial.find_serial
==================

.. code-block::

   (method) serial:find_serial(instance: integer)
     -> AP_Scripting_SerialAccess_ud|nil

----

serial.find_simulated_device
============================

.. code-block::

   (method) serial:find_simulated_device(protocol: integer, instance: integer)
     -> AP_Scripting_SerialAccess_ud|nil

----

serialLED
=========

.. code-block::

   table

----

serialLED.send
==============

.. code-block::

   (method) serialLED:send(chan: integer)
     -> boolean

----

serialLED.set_RGB
=================

.. code-block::

   (method) serialLED:set_RGB(chan: integer, led_index: integer, red: integer, green: integer, blue: integer)
     -> boolean

----

serialLED.set_num_neopixel
==========================

.. code-block::

   (method) serialLED:set_num_neopixel(chan: integer, num_leds: integer)
     -> boolean

----

serialLED.set_num_neopixel_rgb
==============================

.. code-block::

   (method) serialLED:set_num_neopixel_rgb(chan: integer, num_leds: integer)
     -> boolean

----

serialLED.set_num_profiled
==========================

.. code-block::

   (method) serialLED:set_num_profiled(chan: integer, num_leds: integer)
     -> boolean

----

servo_telem
===========

.. code-block::

   table

----

servo_telem.get_telem
=====================

.. code-block::

   (method) servo_telem:get_telem(servo_index: integer)
     -> ServoTelemetryData_ud|nil

----

servo_telem.update_telem_data
=============================

.. code-block::

   (method) servo_telem:update_telem_data(instance: integer, telemdata: ServoTelemetryData_ud)

----

sim
===

.. code-block::

   table

----

sim.set_pose
============

.. code-block::

   (method) sim:set_pose(instance: integer, loc: Location_ud, orient: Quaternion_ud, velocity_bf: Vector3f_ud, gyro_rads: Vector3f_ud)
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

string_to_ipv4_addr
===================

.. code-block::

   function string_to_ipv4_addr(str_address: string)
     -> uint32_t_ud

----

sub
===

.. code-block::

   table

----

sub.get_and_clear_button_count
==============================

.. code-block::

   (method) sub:get_and_clear_button_count(index: integer)
     -> integer

----

sub.get_rangefinder_target_cm
=============================

.. code-block::

   (method) sub:get_rangefinder_target_cm()
     -> number

----

sub.is_button_pressed
=====================

.. code-block::

   (method) sub:is_button_pressed(index: integer)
     -> boolean

----

sub.rangefinder_alt_ok
======================

.. code-block::

   (method) sub:rangefinder_alt_ok()
     -> boolean

----

sub.set_rangefinder_target_cm
=============================

.. code-block::

   (method) sub:set_rangefinder_target_cm(new_target_cm: number)
     -> boolean

----

temperature_sensor
==================

.. code-block::

   table

----

temperature_sensor.get_temperature
==================================

.. code-block::

   (method) temperature_sensor:get_temperature(instance: integer)
     -> number|nil

----

terrain
=======

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

.. code-block::

   (method) terrain:enabled()
     -> boolean

----

terrain.height_above_terrain
============================

.. code-block::

   (method) terrain:height_above_terrain(extrapolate: boolean)
     -> number|nil

----

terrain.height_amsl
===================

.. code-block::

   (method) terrain:height_amsl(loc: Location_ud, corrected: boolean)
     -> number|nil

----

terrain.height_terrain_difference_home
======================================

.. code-block::

   (method) terrain:height_terrain_difference_home(extrapolate: boolean)
     -> number|nil

----

terrain.status
==============

.. code-block::

   (method) terrain:status()
     -> integer

----

uint32_t
========

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

uint64_t
========

.. code-block::

   function uint64_t(value?: number|uint32_t_ud|uint64_t_ud)
     -> uint64_t_ud

.. code-block::

   function uint64_t(high: number|uint32_t_ud, low: number|uint32_t_ud)
     -> uint64_t_ud

----

uint64_t_ud
===========

split
-----

.. code-block::

   (method) uint64_t_ud:split()
     -> uint32_t_ud
     2. uint32_t_ud

 Split into high and low half's, returning each as a uint32_t_ud

@\ *return* — high (value >> 32)

@\ *return* — low (value & 0xFFFFFFFF)

tofloat
-------

.. code-block::

   (method) uint64_t_ud:tofloat()
     -> number

 Convert to number, will loose resolution at large values

toint
-----

.. code-block::

   (method) uint64_t_ud:toint()
     -> integer|nil

 Convert to integer, nil if too large to be represented by native int32

----

vehicle
=======

.. code-block::

   table

----

vehicle.get_circle_radius
=========================

.. code-block::

   (method) vehicle:get_circle_radius()
     -> number|nil

----

vehicle.get_control_mode_reason
===============================

.. code-block::

   (method) vehicle:get_control_mode_reason()
     -> integer

----

vehicle.get_control_output
==========================

.. code-block::

   (method) vehicle:get_control_output(control_output: integer|'1'|'2'|'3'|'4'...(+4))
     -> number|nil

----

vehicle.get_likely_flying
=========================

.. code-block::

   (method) vehicle:get_likely_flying()
     -> boolean

----

vehicle.get_mode
================

.. code-block::

   (method) vehicle:get_mode()
     -> integer

----

vehicle.get_pan_tilt_norm
=========================

.. code-block::

   (method) vehicle:get_pan_tilt_norm()
     -> number|nil
     2. number|nil

----

vehicle.get_steering_and_throttle
=================================

.. code-block::

   (method) vehicle:get_steering_and_throttle()
     -> number|nil
     2. number|nil

----

vehicle.get_target_location
===========================

.. code-block::

   (method) vehicle:get_target_location()
     -> Location_ud|nil

----

vehicle.get_time_flying_ms
==========================

.. code-block::

   (method) vehicle:get_time_flying_ms()
     -> uint32_t_ud

----

vehicle.get_wp_bearing_deg
==========================

.. code-block::

   (method) vehicle:get_wp_bearing_deg()
     -> number|nil

----

vehicle.get_wp_crosstrack_error_m
=================================

.. code-block::

   (method) vehicle:get_wp_crosstrack_error_m()
     -> number|nil

----

vehicle.get_wp_distance_m
=========================

.. code-block::

   (method) vehicle:get_wp_distance_m()
     -> number|nil

----

vehicle.has_ekf_failsafed
=========================

.. code-block::

   (method) vehicle:has_ekf_failsafed()
     -> boolean

----

vehicle.is_crashed
==================

.. code-block::

   (method) vehicle:is_crashed()
     -> boolean

----

vehicle.is_landing
==================

.. code-block::

   (method) vehicle:is_landing()
     -> boolean

----

vehicle.is_taking_off
=====================

.. code-block::

   (method) vehicle:is_taking_off()
     -> boolean

----

vehicle.nav_script_time
=======================

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

.. code-block::

   (method) vehicle:nav_script_time_done(param1: integer)

----

vehicle.nav_scripting_enable
============================

.. code-block::

   (method) vehicle:nav_scripting_enable(param1: integer)
     -> boolean

----

vehicle.reboot
==============

.. code-block::

   (method) vehicle:reboot(hold_in_bootloader: boolean)

----

vehicle.register_custom_mode
============================

.. code-block::

   (method) vehicle:register_custom_mode(number: integer, full_name: string, short_name: string)
     -> AP_Vehicle__custom_mode_state_ud|nil

----

vehicle.set_circle_rate
=======================

.. code-block::

   (method) vehicle:set_circle_rate(rate_dps: number)
     -> boolean

----

vehicle.set_crosstrack_start
============================

.. code-block::

   (method) vehicle:set_crosstrack_start(new_start_location: Location_ud)
     -> boolean

----

vehicle.set_desired_speed
=========================

.. code-block::

   (method) vehicle:set_desired_speed(param1: number)
     -> boolean

----

vehicle.set_desired_turn_rate_and_speed
=======================================

.. code-block::

   (method) vehicle:set_desired_turn_rate_and_speed(param1: number, param2: number)
     -> boolean

----

vehicle.set_land_descent_rate
=============================

.. code-block::

   (method) vehicle:set_land_descent_rate(rate: number)
     -> boolean

----

vehicle.set_mode
================

.. code-block::

   (method) vehicle:set_mode(mode_number: integer)
     -> boolean

----

vehicle.set_rudder_offset
=========================

.. code-block::

   (method) vehicle:set_rudder_offset(rudder_pct: number, run_yaw_rate_control: boolean)

----

vehicle.set_steering_and_throttle
=================================

.. code-block::

   (method) vehicle:set_steering_and_throttle(steering: number, throttle: number)
     -> boolean

----

vehicle.set_target_angle_and_climbrate
======================================

.. code-block::

   (method) vehicle:set_target_angle_and_climbrate(roll_deg: number, pitch_deg: number, yaw_deg: number, climb_rate_ms: number, use_yaw_rate: boolean, yaw_rate_degs: number)
     -> boolean

----

vehicle.set_target_angle_and_rate_and_throttle
==============================================

.. code-block::

   (method) vehicle:set_target_angle_and_rate_and_throttle(roll_deg: number, pitch_deg: number, yaw_deg: number, roll_rate_dps: number, pitch_rate_dps: number, yaw_rate_dps: number, throttle: number)
     -> boolean

----

vehicle.set_target_location
===========================

.. code-block::

   (method) vehicle:set_target_location(target_loc: Location_ud)
     -> boolean

----

vehicle.set_target_pos_NED
==========================

.. code-block::

   (method) vehicle:set_target_pos_NED(target_pos: Vector3f_ud, use_yaw: boolean, yaw_deg: number, use_yaw_rate: boolean, yaw_rate_degs: number, yaw_relative: boolean, terrain_alt: boolean)
     -> boolean

----

vehicle.set_target_posvel_NED
=============================

.. code-block::

   (method) vehicle:set_target_posvel_NED(target_pos: Vector3f_ud, target_vel: Vector3f_ud)
     -> boolean

----

vehicle.set_target_posvelaccel_NED
==================================

.. code-block::

   (method) vehicle:set_target_posvelaccel_NED(target_pos: Vector3f_ud, target_vel: Vector3f_ud, target_accel: Vector3f_ud, use_yaw: boolean, yaw_deg: number, use_yaw_rate: boolean, yaw_rate_degs: number, yaw_relative: boolean)
     -> boolean

----

vehicle.set_target_rate_and_throttle
====================================

.. code-block::

   (method) vehicle:set_target_rate_and_throttle(roll_rate_dps: number, pitch_rate_dps: number, yaw_rate_dps: number, throttle: number)
     -> boolean

----

vehicle.set_target_throttle_rate_rpy
====================================

.. code-block::

   (method) vehicle:set_target_throttle_rate_rpy(param1: number, param2: number, param3: number, param4: number)

----

vehicle.set_target_velaccel_NED
===============================

.. code-block::

   (method) vehicle:set_target_velaccel_NED(target_vel: Vector3f_ud, target_accel: Vector3f_ud, use_yaw: boolean, yaw_deg: number, use_yaw_rate: boolean, yaw_rate_degs: number, yaw_relative: boolean)
     -> boolean

----

vehicle.set_target_velocity_NED
===============================

.. code-block::

   (method) vehicle:set_target_velocity_NED(vel_ned: Vector3f_ud, align_yaw_to_target?: boolean)
     -> boolean

----

vehicle.set_velocity_match
==========================

.. code-block::

   (method) vehicle:set_velocity_match(param1: Vector2f_ud)
     -> boolean

----

vehicle.start_takeoff
=====================

.. code-block::

   (method) vehicle:start_takeoff(alt: number)
     -> boolean

----

vehicle.update_target_location
==============================

.. code-block::

   (method) vehicle:update_target_location(current_target: Location_ud, new_target: Location_ud)
     -> boolean

----

visual_odom
===========

.. code-block::

   table

----

visual_odom.healthy
===================

.. code-block::

   (method) visual_odom:healthy()
     -> boolean

----

visual_odom.quality
===================

.. code-block::

   (method) visual_odom:quality()
     -> integer

----

winch
=====

.. code-block::

   table

----

winch.get_rate_max
==================

.. code-block::

   (method) winch:get_rate_max()
     -> number

----

winch.healthy
=============

.. code-block::

   (method) winch:healthy()
     -> boolean

----

winch.relax
===========

.. code-block::

   (method) winch:relax()

----

winch.release_length
====================

.. code-block::

   (method) winch:release_length(param1: number)

----

winch.set_desired_rate
======================

.. code-block::

   (method) winch:set_desired_rate(param1: number)
