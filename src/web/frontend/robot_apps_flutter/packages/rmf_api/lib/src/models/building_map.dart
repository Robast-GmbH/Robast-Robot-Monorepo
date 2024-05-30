class BuildingMap {
  BuildingMap.fromJson(Map<String, dynamic> json)
      : name = json['name'] as String,
        levels = (json['levels'] as List<dynamic>).map((e) => Level.fromJson(e as Map<String, dynamic>)).toList();

  final String name;
  final List<Level> levels;
}

class Level {
  Level.fromJson(Map<String, dynamic> json)
      : name = json['name'] as String,
        // ignore: avoid_dynamic_calls
        vertices = (json['nav_graphs'][0]['vertices'] as List<dynamic>).map((e) => Vertice.fromJson(e as Map<String, dynamic>)).toList();

  final String name;
  final List<Vertice> vertices;
}

class Vertice {
  Vertice.fromJson(Map<String, dynamic> json)
      : x = json['x'] as double,
        y = json['y'] as double,
        name = json['name'] as String {
    final params = _readParams(json['params'] as List<dynamic>);
    isDispenser = params['pickup_dispenser'] ?? false;
    isIngestor = params['dropoff_ingestor'] ?? false;
  }

  final double x;
  final double y;
  final String name;
  late bool isDispenser;
  late bool isIngestor;

  static Map<String, bool> _readParams(List<dynamic> params) {
    final verticesParams = <String, bool>{};
    for (final param in params) {
      verticesParams[(param as Map<String, dynamic>)['name'] as String] = true;
    }
    return verticesParams;
  }
}

class VerticeParam {
  VerticeParam.fromJson(Map<String, dynamic> json) : name = json['name'] as String;

  final String name;
}
