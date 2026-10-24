# OSMIOR V5.1 — corrección de reproducción y diagnóstico de motores

## 1) Audio: cambio necesario en la microSD

El síntoma de que el número pedido reproduce otra frase suele indicar que `play(n)` está usando el índice de archivo del directorio raíz/FAT y no el número esperado del nombre. El UNO quedó cambiado a `playMp3Folder(n)`, que reproduce por número dentro de la carpeta `MP3`.

**Para que funcione, organiza la microSD así (en mayúsculas `MP3`):**

```text
/MP3/0001.mp3  -> Inicio de misión
/MP3/0002.mp3  -> Batería baja / 15 min restantes
/MP3/0003.mp3  -> Consulta de persona
/MP3/0004.mp3  -> Ángulo máximo / inclinación crítica
/MP3/0005.mp3  -> Persona detectada
/MP3/0006.mp3  -> Inspección de zona
```

Usa cuatro dígitos por nombre (`0001.mp3`, no `001.mp3`) dentro de `/MP3/`. Asegúrate de que cada MP3 contenga solo la frase que corresponde a esa pista. Si el archivo 0006 contiene dos frases juntas, el código no puede separarlas: hay que editar/exportar ese audio o reemplazar ese archivo. Los archivos sueltos llamados `001.mp3` en la raíz no cumplen el nuevo esquema.

Los eventos del firmware permanecen: misión inicia con pista 1; aviso por tiempo/batería con pista 2; botón de consulta persona con pista 3; inclinación crítica con pista 4; detección con pista 5; activar radar con pista 6. La corrección de índice solo da resultado después de organizar los archivos como arriba.

## 2) Motores: se conserva la lógica de producción

Se revisó el mapeo de motores de V3 y V5; ambos tienen la misma tabla lógica. La conducta observada (por ejemplo, giro a la derecha haciendo avanzar ambos lados) no se explica por un simple intercambio global de polaridad. Para evitar empeorar un rover que ya comunica, se incluye `OSMIOR_MOTOR_TEST_V5.ino`, que aísla las salidas del UNO y elimina Nicla, UART, HMI y sensores de la ecuación.

### Prueba aislada obligatoria

1. Desconecta la batería de motores antes de cambiar el firmware. Sube `OSMIOR_MOTOR_TEST_V5.ino` al UNO.
2. Levanta el rover de forma que las seis ruedas queden sin tocar la mesa/suelo. Alimenta y abre Serial Monitor a **115200 baudios**, final de línea **Nueva línea**.
3. Envía una orden cada vez: `F`, `B`, `L`, `R`, `S`. Envía `S` entre pruebas.
4. Resultado esperado: `F` ambos lados hacia adelante; `B` ambos hacia atrás; `L` izquierda hacia atrás/derecha hacia adelante; `R` izquierda hacia adelante/derecha hacia atrás; `S` todos detenidos.

- **Si también falla con este sketch:** el problema está después del software de Nicla/HMI: pinout real D3–D9, puente de entradas, STBY, asignación de motores a salidas A/B, polaridad de cables o driver. Comprueba cada lado y cada motor con batería desconectada antes de volver a cargar la aplicación completa. No unir salidas AO/BO del driver; solo sincronizar las entradas según el esquema existente.
- **Si el sketch aislado funciona:** vuelve a cargar `OSMIOR_UNO_V5_1.ino`, `OSMIOR_NICLA_V5_1.py` y `OSMIOR_HMI_V5_1.py`. El problema está en la cadena de comandos de HMI/Nicla/UART y se debe mirar el último comando que llega al UNO; no cambies pines hasta comprobarlo.

No hagas la prueba con las ruedas tocando el suelo. Un TB6612FNG controla cuatro canales en dos módulos; con seis motores, verifica que la distribución y la corriente por canal sean seguras, especialmente al bloquearse/arrancar.

## 3) Qué se comprobó y qué no

- El cambio de audio es `play(track)` → `playMp3Folder(track)`.
- Los dos archivos Python conservan la lógica V5 y pasan una validación sintáctica si se ejecuta `python -m py_compile`.
- El sketch de diagnóstico es independiente y usa los mismos pines de tracción D3–D9.
- No se hizo prueba física ni compilación Arduino con las librerías instaladas. Verificar primero con ruedas levantadas.
