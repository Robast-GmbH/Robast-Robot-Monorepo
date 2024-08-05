import 'package:middleware_api_utilities/src/models/drawer_address.dart';
import 'package:middleware_api_utilities/src/models/robot_drawer.dart';
import 'package:middleware_api_utilities/src/services/request_service.dart';

class ModulesApi {
  ModulesApi({required this.prefix});
  final String prefix;

  Future<List<RobotDrawer>> getModules({
    required String robotName,
  }) async {
    final drawers = <RobotDrawer>[];
    final response = await RequestService.tryGet(uri: Uri.parse('$prefix/modules?robot_name=$robotName'));
    if (response != null) {
      final data = RequestService.responseToList(response: response);
      for (final drawer in data) {
        drawers.add(RobotDrawer.fromJson(drawer as Map<String, dynamic>));
      }
    }
    return drawers;
  }

  Future<bool> createModule({
    required String robotName,
    required int moduleID,
    required int drawerID,
    required int position,
    required int size,
    required String variant,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/create_module'),
      data: {
        'drawer_address': {
          'robot_name': robotName,
          'module_id': moduleID,
          'drawer_id': drawerID,
        },
        'position': position,
        'size': size,
        'variant': variant,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> deleteModule({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/delete_module'),
      data: {
        'robot_name': robotName,
        'module_id': moduleID,
        'drawer_id': drawerID,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> emptyModule({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/empty_module'),
      data: {
        'robot_name': robotName,
        'module_id': moduleID,
        'drawer_id': drawerID,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> updateModuleContent({
    required String robotName,
    required int moduleID,
    required int drawerID,
    required Map<String, int> itemsByCount,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/update_module_content'),
      data: {
        'drawer_address': {
          'robot_name': robotName,
          'module_id': moduleID,
          'drawer_id': drawerID,
        },
        'items_by_count': itemsByCount,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> freeModule({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/free_module'),
      data: {
        'robot_name': robotName,
        'module_id': moduleID,
        'drawer_id': drawerID,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> reserveModule({
    required String robotName,
    required int moduleID,
    required int drawerID,
    required List<String> userIDs,
    required List<String> userGroups,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/reserve_module'),
      data: {
        'drawer_address': {
          'robot_name': robotName,
          'module_id': moduleID,
          'drawer_id': drawerID,
        },
        'user_ids': userIDs,
        'user_groups': userGroups,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> startModuleProcess({
    required String robotName,
    required DrawerAddress drawerAddress,
    required String processName,
    required Map<String, int> itemsByChange,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/start_module_process'),
      data: {
        'drawer_address': {
          'robot_name': robotName,
          'module_id': drawerAddress.moduleID,
          'drawer_id': drawerAddress.drawerID,
        },
        'process_name': processName,
        'items_by_change': itemsByChange,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> openDrawer({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/open_drawer'),
      data: {
        'robot_name': robotName,
        'module_id': moduleID,
        'drawer_id': drawerID,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> closeDrawer({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/close_drawer'),
      data: {
        'robot_name': robotName,
        'module_id': moduleID,
        'drawer_id': drawerID,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }

  Future<bool> finishModuleProcess({
    required String robotName,
    required int moduleID,
    required int drawerID,
  }) async {
    final response = await RequestService.tryPost(
      uri: Uri.parse('$prefix/modules/finish_module_process'),
      data: {
        'robot_name': robotName,
        'module_id': moduleID,
        'drawer_id': drawerID,
      },
    );
    return RequestService.wasRequestSuccessful(response: response);
  }
}
