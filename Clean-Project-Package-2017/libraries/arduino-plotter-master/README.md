arduino-plotter
===============
An Arduino library for easy graphing on host computer via serial communication

_by: Devin Conley_

---

### Features:
- Continuous multi-variable plots against time
- 2-variable "x" vs "y" plots
- Display multiple graphs within single resizable window
- Support for any data type that can be cast to a double
- Simply pass a reference to your variables when the graph is added, no need to update each value explicitly
- Control number of data points displayed on each graph
- Auto-scaling to fit all data on graph
- Configurable line color per variable
- Stand-alone listener application, written with Processing, is provided

![Plotter Preview](https://www.dropbox.com/s/2mtg5ig7lyrrffi/plotter_preview.gif?raw=1)

---

### Extremely easy usage:
```arduino
#include "Plotter.h"

double x; // global variables
Plotter p;

void setup()
{
  p = Plotter(); // create plotter
  
  p.AddTimeGraph( "Some title of a graph", 500, "label for x", x ); // add any graphs you want
}

void loop()
{
  x = 10*sin( 2.0*PI*( millis() / 5000.0 ) ); // update your variables like usual

  p.Plot(); // plot all current data -- usually called within loop()
}
```

---

### See the Wiki for more information:
[Home](https://github.com/devinaconley/arduino-plotter/wiki)  
[Quickstart](https://github.com/devinaconley/arduino-plotter/wiki/Installation-and-Quickstart)    
[Documentation](https://github.com/devinaconley/arduino-plotter/wiki/Documentation)  
