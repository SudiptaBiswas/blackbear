# Inelastic Rebar

!syntax description /Materials/PlasticTruss

## Description

The `PlasticTruss` model implements J2 plasticity for 1D truss elements. The axial elongation of the element is obtained from
\begin{equation}
\varepsilon = \frac{\Delta l}{l_0} = \frac{(l-l_0)}{l_0} \, ,
\end{equation}
where $\delta l$ is the total stretch computed from the updated length $l$ and the old length $l_0$ of the truss element.
In case of linear elastic material, the axial stress $\sigma_a$ is computed as
\begin{equation}
\sigma_a = E \frac{\Delta l}{l_0} \, .
\end{equation}
Here, the nonlinear behavior of the truss is implemented using a simple J2 plasticity model with linear hardening. The strain increment is obtained from the total stretch update
\begin{equation}
\Delta \varepsilon = \varepsilon  - \varepsilon_{old} \, .
\end{equation}
The trial stress is estimated as
\begin{equation}
\sigma^{tr} = \sigma_{old} + E \Delta \varepsilon \, .
\end{equation}

The yield condition is determined as
\begin{equation}
f = \tilde \sigma^{tr} - E \Delta \varepsilon^p - r -\sigma_y = 0 \, ,
\end{equation}
where $E$ is the Young's modulus, $r$ is the linear hardening function defined as $r=h \varepsilon^p$ with $h$ being the hardening constant, and $\sigma_y$ is the yield stress. When the trial stress is outside of the yield profile the stresses are brought down using the iterative Newton method. The hardening variable at the beginning of the iterative process is obtained as
\begin{equation}
r^{(k)} = r_{old} + h  (\Delta \varepsilon^p)^{(k)} \, .
\end{equation}
The plastic strain increment is computed as
\begin{equation}
d\Delta \varepsilon^p = \frac{f}{\frac{df}{d\Delta \varepsilon^p}} = \frac{(\tilde \sigma^{tr} - E (\Delta \varepsilon^p)^{(k)} - r^k -\sigma_y)}{E + h} = 0 \, .
\end{equation}
The plastic strain for the next iteration is updated
\begin{equation}
(\Delta \varepsilon^p)^{(k+1)} = (\Delta \varepsilon^p)^{(k)} + d\Delta \varepsilon^p \, .
\end{equation}
The iterative process continues until the updated stress lies on the yield curve.
Then the elastic strain is updated as
\begin{equation}
\Delta \varepsilon^e = \Delta \varepsilon - \Delta \varepsilon^p \, .
\end{equation}
The updated axial stress is calculated
\begin{equation}
\sigma_a = \sigma_{old} + E \Delta \varepsilon^e \, .
\end{equation}

## Example Input Syntax

!listing test/tests/rebar/truss_inelastic.i block=Materials/truss

!syntax parameters /Materials/PlasticTruss

!syntax inputs /Materials/PlasticTruss

!syntax children /Materials/PlasticTruss
