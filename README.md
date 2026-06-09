# Voxelización de Mallas Triangulares

Este repositorio contiene la implementación en C++ de un algoritmo de voxelización conservativa sólida basada en octree disperso, inspirado en el artículo *"Fast Parallel Surface and Solid Voxelization on GPUs"* de Schwarz y Seidel (2010).

El repositorio está organizado para trabajar con dos variantes:
- `sequential/`: línea base secuencial.
- `parallel/`: copia estructural de la versión secuencial, preparada para paralelizarse con OpenMP.

## Requisitos Previos

- Compilador de C++ con soporte para C++17 (probado con `g++` / MinGW).
- [CMake](https://cmake.org/) (versión 3.16 o superior).

*Nota: El repositorio ya incluye la librería `happly` para la carga de archivos PLY, por lo que no hay dependencias externas adicionales que instalar.*

## Estructura del Proyecto

- `include/common/`: estructuras y utilidades compartidas.
- `include/io/`: carga de mallas y exportación de formatos.
- `include/sequential/`: interfaz de la versión secuencial.
- `include/parallel/`: interfaz de la versión paralela OpenMP.
- `src/common/`: código compartido (`mesh`, `geometry`, `morton`, `octree`, `timer`).
- `src/io/`: carga de PLY y exportadores (`PLY` puntos, `PLY` superficie, `RAW`).
- `src/sequential/`: implementación secuencial de los 4 algoritmos.
- `src/parallel/`: copia exacta de la implementación secuencial para paralelización con OMP.
- `extern/`: Dependencias externas header-only (e.g., `happly.h`).
- `data/bunny/`: Mallas de prueba.
- `docs/`: Documentos del proyecto y paper base.
- `output/`: Directorio donde se guardan los archivos generados tras la ejecución (PLY/RAW).

## Compilación

Puedes compilar el proyecto fácilmente usando CMake:

```bash
# 1. Crear el directorio de build y generar los archivos de construcción
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release

# 2. Compilar los ejecutables
cmake --build build
```
*(Si usas Visual Studio o Ninja, puedes omitir el `-G "MinGW Makefiles"` o ajustarlo según tu generador preferido).*

## Ejecución y Pruebas (Benchmark)

Los ejecutables se generan en la carpeta `build`. Para ejecutar el programa necesitas especificar la malla de entrada y la resolución deseada (`d`). La resolución debe ser una potencia de 2 y múltiplo de 8 (e.g., `32`, `64`, `128`, `256`, `512`).

### Uso básico

```bash
./build/voxelizer_seq <ruta_al_archivo.ply> <resolucion>
./build/voxelizer_omp <ruta_al_archivo.ply> <resolucion>
```

### Opciones de Exportación

Puedes exportar los vóxeles resultantes para visualizar o procesar externamente:
- `--export-ply <archivo.ply>`: Genera una nube de puntos que puede abrirse en programas como MeshLab.
- `--export-voxel-ply <archivo.ply>`: Genera solo las caras externas de los vóxeles como malla triangular.
- `--export-raw <archivo.raw>`: Genera un arreglo binario denso de $d \times d \times d$ bytes (1 para sólido, 0 para vacío).

### Ejemplos

**Ejecución con resolución 256 y exportación a PLY:**
```bash
./build/voxelizer_seq data/bunny/reconstruction/bun_zipper.ply 256 --export-ply output/bunny_256.ply
```

**Ejecución con resolución 256 y exportación de superficie voxelizada:**
```bash
./build/voxelizer_omp data/bunny/reconstruction/bun_zipper.ply 256 --export-voxel-ply output/bunny_256_surface.ply
```

**Ejecución con resolución 512, exportando PLY y RAW:**
```bash
./build/voxelizer_seq data/bunny/reconstruction/bun_zipper.ply 512 --export-ply output/bunny_512.ply --export-raw output/bunny_512.raw
```

## Salida de Consola

El programa imprimirá un desglose de los tiempos tomados por cada uno de los 4 algoritmos, el tiempo total secuencial ($T_s$), y la cantidad de vóxeles sólidos generados. Esta información es ideal para reportar y graficar tus benchmarks.
