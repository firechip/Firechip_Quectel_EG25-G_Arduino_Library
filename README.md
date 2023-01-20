Firechip Quectel EG25-G Arduino Library
==============================

[![Project Status: WIP – Initial development is in progress, but there has not yet been a stable, usable release suitable for the public.](https://www.repostatus.org/badges/latest/wip.svg)](https://www.repostatus.org/#wip)
[![Report Size Deltas](https://github.com/firechip/Firechip_Quectel_EG25-G_Arduino_Library/actions/workflows/report-size-deltas.yml/badge.svg)](https://github.com/firechip/Firechip_Quectel_EG25-G_Arduino_Library/actions/workflows/report-size-deltas.yml)
[![Compile Examples](https://github.com/firechip/Firechip_Quectel_EG25-G_Arduino_Library/actions/workflows/compile-examples.yml/badge.svg)](https://github.com/firechip/Firechip_Quectel_EG25-G_Arduino_Library/actions/workflows/compile-examples.yml)
[![CodeQL](https://github.com/firechip/Firechip_Quectel_EG25-G_Arduino_Library/actions/workflows/codeql.yml/badge.svg)](https://github.com/firechip/Firechip_Quectel_EG25-G_Arduino_Library/actions/workflows/codeql.yml)
[![Check Arduino](https://github.com/firechip/Firechip_Quectel_EG25-G_Arduino_Library/actions/workflows/check-arduino.yml/badge.svg)](https://github.com/firechip/Firechip_Quectel_EG25-G_Arduino_Library/actions/workflows/check-arduino.yml)
[![Spell Check](https://github.com/firechip/Firechip_Quectel_EG25-G_Arduino_Library/actions/workflows/spell-check.yml/badge.svg)](https://github.com/firechip/Firechip_Quectel_EG25-G_Arduino_Library/actions/workflows/spell-check.yml)


<table class="table table-hover table-striped table-bordered">
    <tr align="center">
      <td><a href="https://firechip.dev/products/17272"><img src="https://cdn.firechip.com/assets/parts/1/6/2/7/9/17272-Firechip_MicroMod_Asset_Tracker_Carrier_Board-01a.jpg" alt="MicroMod Asset Tracker Carrier Board"></a></td>
      <td><a href="https://firechip.dev/products/18031"><img src="https://cdn.firechip.com/assets/parts/1/7/2/6/0/18031-Firechip_LTE_GNSS_Breakout_-_EG25-G-01.jpg" alt="Firechip LTE GNSS Breakout - EG25-G"</a></td>
    </tr>
    <tr align="center">
      <td><a href="https://firechip.dev/products/17272">MicroMod Asset Tracker Carrier Board</a></td>
      <td><a href="https://firechip.dev/products/18031">LTE GNSS Breakout - EG25-G</a></td>
    </tr>
</table>
       
An Arduino library for the Quectel EG25-G LTE-M / NB-IoT modules with secure cloud, as used on the [Firechip MicroMod Asset Tracker](https://firechip.dev/products/17272) and the [Firechip LTE GNSS Breakout - EG25-G](https://firechip.dev/products/18031).

v1.1 has had a thorough update and includes new features and examples. This library now supports up to 7 simultaneous TCP or UDP sockets. There are new examples to show how to play ping pong with multiple TCP and UDP sockets.

v1.1 also supports binary data transfers correctly. There are new examples showing how you can integrate this library with the [Firechip Quectel GNSS Arduino Library](https://github.com/firechip/Firechip_Quectel_GNSS_Arduino_Library) and use the EG25-G to: download AssistNow Online and Offline data and push it to the GNSS; open a connection to a NTRIP Caster (such as RTK2go, Skylark or Emlid Caster) and push RTK correction data to the GNSS.

You can install this library using the Arduino IDE Library Manager: search for _**Firechip Quectel EG25-G**_

## Repository Contents

* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE.
* **/src** - Source files for the library (.cpp, .h).
* **keywords.txt** - Keywords from this library that will be highlighted in the Arduino IDE.
* **library.properties** - General library properties for the Arduino package manager.

## Contributing

If you would like to contribute to this library: please do, we truly appreciate it, but please follow [these guidelines](./CONTRIBUTING.md). Thanks!

## Documentation

* **[Installing an Arduino Library Guide](https://learn.firechip.com/tutorials/installing-an-arduino-library)** - Basic information on how to install an Arduino library.
* **[MicroMod Asset Tracker Hookup Guide](https://learn.firechip.com/tutorials/micromod-asset-tracker-carrier-board-hookup-guide)** - Hookup Guide for the MicroMod Asset Tracker Carrier Board.
* **[MicroMod Asset Tracker Product Repository](https://github.com/firechip/MicroMod_Asset_Tracker)** - MicroMod Asset Tracker repository (including hardware files).
* **[Firechip LTE GNSS Breakout - EG25-G Hookup Guide](https://learn.firechip.com/tutorials/lte-gnss-breakout---sara-r5-hookup-guide)** - Hookup Guide for the LTE GNSS Breakout - EG25-G.
* **[Firechip LTE GNSS Breakout - EG25-G Product Repository](https://github.com/firechip/Firechip_LTE_GNSS_Breakout_EG25-G10M8S)** - LTE GNSS Breakout - EG25-G repository (including hardware files).
* **[LICENSE.md](./LICENSE.md)** - License Information

## Products that use this library

* **[DEV-17272](https://firechip.dev/products/17272)** - MicroMod Asset Tracker
* **[GPS-18031](https://firechip.dev/products/18031)** - Firechip LTE GNSS Breakout - EG25-G

- Your friends at Firechip.
