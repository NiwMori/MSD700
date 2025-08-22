[**English**](README.en.md)  | [日本語](README.md)

## Additional Features

- Implements actuator control with slowdown zones based on feedback input.
- Applies safety zones that restrict or pulse actuator movement depending on the actuator’s position.  


## Slowdown Zones

- **Zone 1 (0–5%)** → Force forward slow (pulsed).  
- **Zone 1.5 (5–6%)** → Forward allowed, reverse prohibited.  
- **Zone 2 (6–20%)** → Forward normal, reverse slowed (pulsed).  
- **Normal (>20%)** → Full forward and reverse allowed.  

## Pulsed Slowdown Control
- Motion in restricted zones is modulated using a **100 ms ON/OFF cycle**.  
