# Voxelización de Mallas Triangulares

Este repositorio contiene la implementación en C++ de un algoritmo de voxelización conservativa sólida basada en octree disperso, inspirado en el artículo *"Fast Parallel Surface and Solid Voxelization on GPUs"* de Schwarz y Seidel (2010).

El repositorio está organizado para trabajar con dos variantes:
- `sequential/`: línea base secuencial.
- `parallel/`: copia estructural de la versión secuencial, preparada para paralelizarse con OpenMP.

## Requisitos Previos

- Compilador de C++ con soporte para C++17 (probado con `g++` / MinGW).
- [CMake](https://cmake.org/) (versión 3.16 o superior).

## Compilación

Puedes compilar el proyecto fácilmente usando CMake:

```bash
# 1. Crear el directorio de build y generar los archivos de construcción
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release

# 2. Compilar los ejecutables
cmake --build build
```

## Ejecución y Pruebas (Benchmark)

Los ejecutables se generan en la carpeta `build`. Para ejecutar el programa necesitas especificar la malla de entrada y la resolución deseada (`d`). La resolución debe ser una potencia de 2 y múltiplo de 8 (e.g., `32`, `64`, `128`, `256`, `512`).

### Formato de Entrada

- El programa acepta únicamente mallas triangulares en formato `OBJ`.
- Los archivos de entrada deben ubicarse dentro de `data/input`.

#### Modelos Disponibles en `data/input`

La siguiente tabla muestra la cantidad de triángulos efectivos procesados por el proyecto para cada modelo `OBJ`:

| Modelo | Triángulos |
| --- | ---: |
| `torus.obj` | 576 |
| `sphere.obj` | 1920 |
| `bull.obj` | 2415 |
| `cow.obj` | 5804 |
| `spot.obj` | 5856 |
| `homer.obj` | 12000 |
| `bimba.obj` | 46220 |
| `kar.obj` | 50000 |
| `beast.obj` | 64618 |
| `horse.obj` | 96966 |
| `happy.obj` | 98601 |
| `nefertiti.obj` | 99938 |
| `lucy.obj` | 99970 |
| `xyzrgb_dragon.obj` | 249882 |
| `armadillo.obj` | 345944 |

### Uso básico

```bash
./build/voxelizer_seq <ruta_al_archivo.obj> <resolucion>
./build/voxelizer_omp <ruta_al_archivo.obj> <resolucion>
```

### Formato de Salida

- La salida principal del proyecto es el formato `VTU` (`vtkUnstructuredGrid`).
- Los archivos generados se guardan dentro de `data/output`.
- Opción disponible:
  - `--export-octree-vtu <archivo.vtu>`: Exporta el volumen voxelizado como bloques adaptativos organizados por un octree disperso.

### Visualización de Resultados

Para visualizar los resultados del algoritmo se está utilizando **ParaView**. Esta aplicación permite abrir directamente archivos `VTU` (`vtkUnstructuredGrid`) y resulta adecuada para inspeccionar el volumen voxelizado generado por el proyecto.

Con ParaView es posible:
- visualizar el modelo voxelizado en 3D
- colorear las celdas por atributos como `cell_span`, `depth` o `level`
- aplicar cortes (`Clip`) para inspeccionar el interior del volumen
- verificar que la salida corresponde a un volumen organizado por un octree disperso y no solo a una superficie

Sitio oficial de ParaView:
- https://www.paraview.org/

### Ejemplos

**Ejecución secuencial con un modelo OBJ y exportación a VTU:**
```bash
./build/voxelizer_seq data/input/cow.obj 64 --export-octree-vtu cow_64_sparse.vtu
```

**Ejecución paralela con un modelo OBJ y exportación a VTU:**
```bash
./build/voxelizer_omp data/input/spot.obj 128 --export-octree-vtu spot_128_sparse.vtu
```

## Salida de Consola

El programa imprimirá un desglose de los tiempos tomados por cada uno de los 4 algoritmos, el tiempo total secuencial ($T_s$), y la cantidad de vóxeles sólidos generados. Esta información es ideal para reportar y graficar tus benchmarks.
