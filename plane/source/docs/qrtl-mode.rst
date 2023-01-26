.. _qrtl-mode:

=========
QRTL Mode
=========

QRTL mode (QuadPlane Return To Launch mode) navigates QuadPlane from its current
position to hover above the home position and then land. The behavior of QRTL mode can
be controlled by several adjustable parameters. This page describes how
to use and customize QRTL mode.

Overview
========

When QRTL mode is selected, the QuadPlane will return to the home location. By default, it will transition to fixed wing mode (if in a VTOL mode before entering, it will climb to :ref:`Q_RTL_ALT<Q_RTL_ALT>` if below that altitude, before transitioning), executing the first part of a normal RTL, and then make an approach  as it nears the landing point and switch to VTOL mode and proceed to the landing point, then descend to a landing. See the description of this under the :ref:`Hybrid RTL<hybrid_rtl>` section for :ref:`Q_RTL_MODE<Q_RTL_MODE>` = 3.

If a pure VTOL QRTL is desired, then you must disable the fixed wing RTL and approach feature by setting :ref:`Q_OPTIONS<Q_OPTIONS>` bit 16. Then the following actions will then occur on a QRTL:

The QuadPlane will immediately navigate towards the home location at :ref:`Q_WP_SPEED<Q_WP_SPEED>`, climbing or descending towards the :ref:`Q_RTL_ALT<Q_RTL_ALT>` altitude. Once arriving within :ref:`Q_WP_RADIUS<Q_WP_RADIUS>` distance of home, it will begin descending at :ref:`Q_WP_SPEED_DN<Q_WP_SPEED_DN>` rate, until it reaches :ref:`Q_LAND_FINAL_ALT<Q_LAND_FINAL_ALT>` at which point it will descend at :ref:`Q_LAND_SPEED<Q_LAND_SPEED>` until landing.

.. image:: ../images/QRTL.jpg
    :target: ../_images/QRTL.jpg

.. note::

    QuadPlane will recognize that it has landed if the motors are at
    minimum but its altitude does not change more than 0.2m for one
    second.  It does not use the altitude itself to decide whether to shut off the
    motors except that the QuadPlane must also be below :ref:`Q_LAND_FINAL_ALT<Q_LAND_FINAL_ALT>` above home(ie in the LAND FINAL phase). The altitude change for the decision can be increased, in case the altitude determination from the EKF is excessively noisy by increasing the :ref:`Q_LAND_ALTCHG<Q_LAND_ALTCHG>` value from its default value of 0.2m.
    

Alternatively, you may :ref:`configure the plane to return to a Rally Point <common-rally-points>`, rather than the home location.

.. warning::

   "Home" position is always supposed to be your Plane's actual
   GPS takeoff location:

   #. It is very important to acquire GPS lock before arming in order for
      QRTL, QLOITER, AUTO or any GPS dependent mode to work properly. This requirement is provided in  the default arming checks. It is highly recommended that this check is not disabled.
   #. For QuadPlane the home position is initially established at the time the
      plane acquires its GPS lock. It is then continuously updated as long as
      the autopilot is disarmed.

      - This means if you execute an QRTL in QuadPlane, it will return to the
	location where it was when it was armed - assuming it had
	acquired GPS lock.
      - Consider the use of :ref:`Rally Points <common-rally-points>` to
	avoid returning directly to your arming point on QRTL


.. warning::

   In QRTL mode the autopilot uses a barometer which
   measures air pressure as the primary means for determining altitude
   ("Pressure Altitude") and if the air pressure is changing in your flight
   area, the copter will follow the air pressure change rather than actual
   altitude.


Notes
=====


-  Landing and re-arming the QuadPlane will reset home, which is a great
   feature for flying at airfields.
-  If you get lock for the first time while flying, your home will be
   set at the location of lock.


Return Altitude
===============

If in a VTOL motors are active when QRTL is entered the vehicle will first climb vertically before returning home. The altitude to which the vehicle climbs depends on the distance from home as shown below.
Altitude will be above terrain if terrain following is enabled, if not altitude will be above home.

.. raw:: html
   <embed>

   <!DOCTYPE html>
   <html lang="en">
   <head>
   <meta charset="utf-8" />
   <title>QRTL Altitude Interactive</title>
   <script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/2.9.4/Chart.js"></script>
   </head>
   <body onload="update();">
   <canvas id="QRTL_Altitude" style="width:100%;max-width:1200px"></canvas>
      <p>
         <label for="Q_RTL_ALT">Q_RTL_ALT</label>
         <input id="Q_RTL_ALT" name="Q_RTL_ALT" type="number" step="1" value="15" min="0" onchange="update();"/>
      </p>
      <p>
         <label for="Q_RTL_ALT_MIN">Q_RTL_ALT_MIN</label>
         <input id="Q_RTL_ALT_MIN" name="Q_RTL_ALT_MIN" type="number" step="1" value="10.0" min="0" onchange="update();"/>
      </p>
      <p>
         <label for="RTL_RADIUS">RTL_RADIUS</label>
         <input id="RTL_RADIUS" name="RTL_RADIUS" type="number" step="1" value="0" onchange="update();"/>
      </p>
      <p>
         <label for="WP_LOITER_RAD">WP_LOITER_RAD</label>
         <input id="WP_LOITER_RAD" name="WP_LOITER_RAD" type="number" step="1" value="60" onchange="update();"/>
      </p>
      <p>
         <label for="Q_LAND_FINAL">Q_LAND_FINAL</label>
         <input id="Q_LAND_FINAL" name="Q_LAND_FINAL" type="number" step="0.5" value="6" min="0" onchange="update();"/>
      </p>

   <script>
   var chart;
   function update() {
      var Q_RTL_ALT = parseFloat(document.getElementById("Q_RTL_ALT").value)
      var Q_RTL_ALT_MIN = parseFloat(document.getElementById("Q_RTL_ALT_MIN").value)
      var RTL_RADIUS = parseFloat(document.getElementById("RTL_RADIUS").value)
      var WP_LOITER_RAD = parseFloat(document.getElementById("WP_LOITER_RAD").value)
      var Q_LAND_FINAL = parseFloat(document.getElementById("Q_LAND_FINAL").value)

      if ((Q_LAND_FINAL + 1) > Q_RTL_ALT) {
         Q_RTL_ALT = Q_LAND_FINAL + 1
         document.getElementById("Q_RTL_ALT").value = Q_RTL_ALT
      }

      if (Q_RTL_ALT_MIN < Q_LAND_FINAL) {
         Q_RTL_ALT_MIN = Q_LAND_FINAL
         document.getElementById("Q_RTL_ALT_MIN").value = Q_LAND_FINAL
      } else if (Q_RTL_ALT_MIN > Q_RTL_ALT) {
         Q_RTL_ALT_MIN = Q_RTL_ALT
         document.getElementById("Q_RTL_ALT_MIN").value = Q_RTL_ALT
      }

      var radius = Math.max(Math.abs(RTL_RADIUS), Math.abs(WP_LOITER_RAD)) * 1.5
      var min_alt_radius = (Q_RTL_ALT_MIN/Q_RTL_ALT) * radius;

      var max_disp_rad = Math.ceil((radius * 1.2) / 10) * 10
      var max_disp_alt = Math.ceil((Q_RTL_ALT+5.0) / 10) * 10

      var alt = [{x:-5,             y:Q_RTL_ALT_MIN},
                 {x:min_alt_radius, y:Q_RTL_ALT_MIN},
                 {x:radius,         y:Q_RTL_ALT},
                 {x:max_disp_rad+5, y:Q_RTL_ALT}]

      var land_final = [{x:-5,             y:Q_LAND_FINAL},
                        {x:max_disp_rad+5, y:Q_LAND_FINAL}]

      if (chart == null) {
         chart = new Chart("QRTL_Altitude", {
            type : "scatter",
            data: {
                  datasets: [
                     {
                        label: 'Return Altitude',
                        borderColor: "rgba(0,0,255,1.0)",
                        pointBackgroundColor: "rgba(0,0,255,1.0)",
                        data: alt,
                        fill: false,
                        showLine: true,
                        lineTension: 0,
                     },
                     {
                        label: 'Q_LAND_FINAL',
                        borderColor: "rgba(0,255,0,0.25)",
                        pointBackgroundColor: "rgba(0,255,0,0.25)",
                        data: land_final,
                        fill: false,
                        showLine: true,
                        lineTension: 0,
                     }
                  ]
            },
            options: {
                  aspectRatio: 3,
                  scales: {
                     yAxes: [
                        {
                              scaleLabel: { display: true, labelString: "Altitude (m)" },
                              ticks: {min:0.0, max:max_disp_alt}
                        },
                     ],
                     xAxes: [
                        {
                              scaleLabel: { display: true, labelString: "Distance from home (m)" },
                              ticks: { min:0.0, max:max_disp_rad}
                        }
                     ],
                  }
            }
         });
      } else {
         chart.data.datasets[0].data = alt;
         chart.data.datasets[1].data = land_final;
         chart.options.scales.xAxes[0].ticks.max = max_disp_rad;
         chart.options.scales.yAxes[0].ticks.max = max_disp_alt;
         chart.update();
      }
   }
   </script>

   </body>
   </html>

   </embed>
