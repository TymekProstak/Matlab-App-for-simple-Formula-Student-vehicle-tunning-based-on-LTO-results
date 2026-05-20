# LTO MATLAB App — Formula Student

Skeleton projektu pod aplikację MATLAB App Designer do interaktywnego LTO / minimum lap time optimization.

Na tym etapie projekt zawiera tylko warstwę GUI/helperów:

- wczytywanie konfiguracji JSON,
- wczytywanie toru CSV w formacie left/center/right,
- obliczanie geometrii toru: `s`, `ds`, `curvature`, szerokości,
- rysowanie toru w `uiaxes`,
- dummy solver generujący przykładowy profil prędkości i GG diagram,
- helper do przeliczania wheel rate + sprung mass + damping ratio na częstotliwości i stałe czasowe.

Właściwa fizyka LTO, CasADi i IPOPT będą podpinane później w `src/solvers/` i `src/models/`.

## Sugerowany workflow w MATLAB

1. Otwórz MATLAB w folderze `LTO_MATLAB_APP`.
2. Uruchom:

```matlab
startup
```

3. Otwórz App Designer:

```matlab
appdesigner
```

4. Utwórz nową aplikację `app/LTOApp.mlapp`.
5. W App Designer dodaj komponenty opisane w `app/LTOApp_AppDesigner_Callbacks_Template.m`.
6. Wklej properties, startupFcn i callbacki z template'u.

## Format toru CSV

Wymagane kolumny:

```text
x_left_m,y_left_m,x_center_m,y_center_m,x_right_m,y_right_m
```

Opcjonalnie można dodać:

```text
s_m,curvature_1pm
```

Jeżeli `s_m` lub `curvature_1pm` nie ma w pliku, aplikacja policzy je automatycznie.

## Najważniejsza decyzja modelowa

Nie wpisujemy `tau_lat_elastic` i `tau_long_elastic` ręcznie jako magicznych stałych. W konfiguracji podajemy parametry fizyczne:

- wheel rate front/rear,
- sprung mass front/rear,
- damping ratio,
- roll frequency factor.

Z nich funkcja `computeSuspensionDerivedParams()` wylicza częstotliwości i stałe czasowe używane później przez model LTO.

