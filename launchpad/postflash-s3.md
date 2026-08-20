# S3 weather front end — flashed

This chip owns Wi-Fi: it geocodes your location, fetches the forecast from
Open-Meteo and feeds it to the H2 over the inter-chip UART.

Flashing erased NVS, so **any Wi-Fi and location you'd saved before is gone** and
needs entering again.

## Set it up in the Console tab

Open the **Console** tab, press **Reset Device**, and you'll get a `weather>`
prompt. Type these one at a time — the browser console sends whole lines, which
is exactly what these commands expect:

```
wifi "my network" "my password"     quote anything containing spaces
location Melbourne                  searches, and lists numbered matches
pick 2                              choose one of them (skip if only one matched)
units c                             or f — see the note below
refresh 60                          minutes between fetches
save                                connects, fetches, and prints the result
```

`show` prints the current configuration, `poll` fetches on demand, and `help`
lists everything.

`location` needs Wi-Fi, so set `wifi` first — it brings the radio up and waits
about 15 seconds before searching.

> There's also a full-screen arrow-key `setup` page, but it needs a real terminal
> (`idf.py monitor`, `screen`, PuTTY). Browser consoles can't send arrow keys, so
> use the line commands here. If you do end up stuck in it, type `q` then Enter.

## Units

Matter's temperature field is nominally Celsius, but the display just renders the
number it receives — so `units f` publishes °F directly. A third-party controller
reading the raw attribute would interpret it as °C.

## Next

Flash `h2_matter` through the board's **other** USB-C port if you haven't, then
commission the H2 in your Matter app and bind the display to it.
