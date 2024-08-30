import 'package:middleware_api_utilities/src/models/submodule.dart';
import 'package:middleware_api_utilities/src/models/submodule_address.dart';
import 'package:middleware_api_utilities/src/services/request_service.dart';

class ModulesApi {
  ModulesApi({required this.prefix});
  final String prefix;

  Future<List<Submodule>> getSubmodules({
    required String robotName,
  }) async {
    final submodules = <Submodule>[];
    final response = await RequestService.tryGet(uri: Uri.parse('$prefix/modules?robot_name=$robotName'));
    if (response != null) {
      final data = RequestService.responseToList(response: response);
      for (final submodule in data) {
        submodules.add(Submodule.fromJson(submodule as Map<String, dynamic>));
      }
    }
    return submodules;
  }

  Future<bool> createSubmodule({
    required String robotName,
    required SubmoduleAddress submoduleAddress,
    required int position,
    required int size,
    required String variant,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/create_submodule'),
      data: {
        'submodule_address': {
          'robot_name': robotName,
          'module_id': submoduleAddress.moduleID,
          'submodule_id': submoduleAddress.submoduleID,
        },
        'position': position,
        'size': size,
        'variant': variant,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> deleteSubmodule({required String robotName, required SubmoduleAddress submoduleAddress}) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/delete_submodule'),
      data: {
        'robot_name': robotName,
        'module_id': submoduleAddress.moduleID,
        'submodule_id': submoduleAddress.submoduleID,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> emptySubmodule({
    required String robotName,
    required SubmoduleAddress submoduleAddress,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/empty_submodule'),
      data: {
        'robot_name': robotName,
        'module_id': submoduleAddress.moduleID,
        'submodule_id': submoduleAddress.submoduleID,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> updateSubmoduleContent({
    required String robotName,
    required SubmoduleAddress submoduleAddress,
    required Map<String, int> itemsByCount,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/update_submodule_content'),
      data: {
        'submodule_address': {
          'robot_name': robotName,
          'module_id': submoduleAddress.moduleID,
          'submodule_id': submoduleAddress.submoduleID,
        },
        'items_by_count': itemsByCount,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> freeSubmodule({
    required String robotName,
    required SubmoduleAddress submoduleAddress,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/free_submodule'),
      data: {
        'robot_name': robotName,
        'module_id': submoduleAddress.moduleID,
        'submodule_id': submoduleAddress.submoduleID,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> reserveSubmodule({
    required String robotName,
    required SubmoduleAddress submoduleAddress,
    required String taskID,
    required List<String> userIDs,
    required List<String> userGroups,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/reserve_submodule'),
      data: {
        'submodule_address': {
          'robot_name': robotName,
          'module_id': submoduleAddress.moduleID,
          'submodule_id': submoduleAddress.submoduleID,
        },
        'task_id': taskID,
        'user_ids': userIDs,
        'user_groups': userGroups,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> startSubmoduleProcess({
    required String robotName,
    required SubmoduleAddress submoduleAddress,
    required String processName,
    required Map<String, int> itemsByChange,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/start_submodule_process'),
      data: {
        'submodule_address': {
          'robot_name': robotName,
          'module_id': submoduleAddress.moduleID,
          'submodule_id': submoduleAddress.submoduleID,
        },
        'process_name': processName,
        'items_by_change': itemsByChange,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> openSubmodule({
    required String robotName,
    required SubmoduleAddress submoduleAddress,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/open_submodule'),
      data: {
        'robot_name': robotName,
        'module_id': submoduleAddress.moduleID,
        'submodule_id': submoduleAddress.submoduleID,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> closeSubmodule({
    required String robotName,
    required SubmoduleAddress submoduleAddress,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/close_submodule'),
      data: {
        'robot_name': robotName,
        'module_id': submoduleAddress.moduleID,
        'submodule_id': submoduleAddress.submoduleID,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> finishSubmoduleProcess({
    required String robotName,
    required SubmoduleAddress submoduleAddress,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/finish_submodule_process'),
      data: {
        'robot_name': robotName,
        'module_id': submoduleAddress.moduleID,
        'submodule_id': submoduleAddress.submoduleID,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }
}
