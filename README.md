# -Desarrollo-de-actuadores-altamente-din-micos-con-aplicaciones-en-rob-tica-bio-inspirada
# Controlador PID de posicion y velocidad
  1. En el archivo "PID_cascada", se encuntra el proyecto realizado en Code Composer Studio de la programacion del PID en cascada, individual, termino feedforward para 
     la primera fase del proyecto.
  2. En el archivo "Resulatados de controlador PID", se encuntra los datos obtenidos de la comunicaicon UART al momento de estar realizando pruebas.
     En el arvhico "mejores resultados obtenido" dento de este se encuntran los resultados utilizados.
  4. Se encuntran el archivo de Simulink utilizado para la comparacion de matlab y los resultados obtenidos con la TIVA C.
  5. Se encuntra el archivo de Excel con el que se realizo la regresion lineal para estimar el termino feedforward.
  6. Se encutra el archivo de matlab para realizar comunicacion serial UART con la tiva C.
# Controlador moderno LQI
  1. En el archivo "Planteamiento_de_optimizacion_y_controlador_LQI", se encuntra almacenado la fase del proyecto de control moderno mediante LQI.
  2. En el folder "CONTRO_LQI", Esta almacenado el proyecto realizado en Code Composer Studio, que contiene la programacion del este controlador para la Tiva C.
  3. En el folder "Datos recopilado optimizacion", estan almacenadas diversas pruebas realizada de la optimizacion para plantear el contralor LQI en matlab.
     En el folder "optimizacion" dentro de este, se encuentra los datos "z_carga", los cuales son los que validaron el controlador LQI
  4. Nueva mente en el folder "Planteamiento_de_optimizacion_y_controlador_LQI", se encuentra un floder llamado "Optimizacion". En este se encuntra el codigo realizado
     de la optimizacion.
  5. En el folder "Piezas 3D", se encuntran todos los diseños realizados de piezas 3D para las pruebas realizadas de controlador.
  6. En el folder "Resultados obtenidos", estan los datos para las graficas de los resultados finales obtenido.
  7. El archivo UART.m, es el que permite realizar la comunicacion UART directamente con la TIVA C y matlab.
  8. El archivo "simulacion.m", es el archivo con el cual se planteo el controlador LQI y se valido que este era implementable teoricamente.
 # Si es prueban los archivos con los resultados obtenido de matalb, ajustar el vector de tiempo y la contrante k para que se puedan graficar los resultados 
 # Los videos de los resultados obtenidos se encuentran en el siguiente link: https://drive.google.com/drive/folders/1iSk8lgw2Ade7hkjXQ5iXjq29X_LL3meq?usp=sharing
