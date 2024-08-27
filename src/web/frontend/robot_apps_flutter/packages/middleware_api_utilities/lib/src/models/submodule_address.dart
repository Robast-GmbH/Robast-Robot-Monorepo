import 'package:meta/meta.dart';

@immutable
class SubmoduleAddress {
  const SubmoduleAddress({
    required this.moduleID,
    required this.submoduleID,
  });

  final int moduleID;
  final int submoduleID;

  @override
  bool operator ==(Object other) {
    return other is SubmoduleAddress && other.moduleID == moduleID && other.submoduleID == submoduleID;
  }

  @override
  int get hashCode => Object.hashAll([moduleID, submoduleID]);

  Map<String, dynamic> toJson() {
    return {
      'module_id': moduleID,
      'submodule_id': submoduleID,
    };
  }
}
