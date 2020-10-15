# extremum-seeking-wecs
Control optimization of point absorber wave energy converter devices using an extremum-seeking approach

Extremum-seeking (ES) codes used to generate results for

L. Parrinello, P. Dafnakis, E.Pasta, G. Bracco, P. Naseradinmousavi, G. Mattiazzo, A.P.S. Bhalla. "An adaptive and energy-maximizing control optimization of wave energy converters using an extremum-seeking approach". https://arxiv.org/abs/2007.04077 

The code simulates the mass-spring-damper (MSD) and point absorber (PA) systems in presence of external forcing and regular/irregular waves, respectively. Energy-maximizing power-take-off (PTO) mechanism coefficients, K and C, using the proportional-derivative (PD) control law are obtained using:

- Sliding mode;
- Self-driving;
- Relay;
- Least-squares; and 
- Perturbation-based extremum-seeking methods.


Abbreviations used in the script names:

- K := Stiffness coefficient of the PTO control force. If present in the name, the script optimizes K coefficient
- C/B := Damping coefficient of the PTO control force. If present in the name, the script optimizes C coefficient
- tot: = In this model, the instantaneous power is defined to sum of both reactive and resistive power components 
- MSD := Mass-Spring-Damper
- MA := Moving Average
- 1D = 1 DOF (Heave)
- self_driving := Self driving ES algorithm is used in the model
- SM/SMESC := Sliding mode algorithm is used in the model
- classic log := Perturbation-based ES is used in the model
- dither free := Relay-based ES is used in the model
- LSQ := Least squares method ES is used in the model
