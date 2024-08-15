import 'package:meta/meta.dart';

@immutable
class DrawerAddress {
  const DrawerAddress({
    required this.moduleID,
    required this.drawerID,
  });

  final int moduleID;
  final int drawerID;

  @override
  bool operator ==(Object other) {
    return other is DrawerAddress && other.moduleID == moduleID && other.drawerID == drawerID;
  }

  @override
  int get hashCode => Object.hashAll([moduleID, drawerID]);

  Map<String, dynamic> toJson() {
    return {
      'module_id': moduleID,
      'drawer_id': drawerID,
    };
  }
}
