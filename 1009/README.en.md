[**English**](README.en.md)  | [日本語](README.md)

## Additional Features

- Implements actuator control with slowdown zones based on feedback input.
- Applies safety zones that restrict or pulse actuator movement depending on the actuator’s position.  


## Slowdown Zones

- **Zone 1 (0–2.5%)** → Force forward slow (pulsed).  
- **Zone 1.5 (2.5–4.5%)** → Forward allowed, reverse prohibited.  
- **Zone 2 (4.5–30%)** → Forward normal, reverse slowed (pulsed).  
- **Normal (>10%)** → Full forward and reverse allowed.  

## Pulsed Slowdown Control
- Motion in restricted zones is modulated using a **100 ms ON/OFF cycle**.  
