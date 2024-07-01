// ignore_for_file: prefer_const_constructors
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:middleware_api_utilities/src/sub_apis/modules_api.dart';
import 'package:test/test.dart';

void main() {
  group('MiddlewareApiUtilities', () {
    const robotName = 'rb_theron';
    const moduleID = 7;
    const drawerID = 0;
    const position = 9;
    const size = 1;
    const itemID = 'Stifte';
    const quantity = 3;
    const variant = 'manual';
    final modulesApi = ModulesApi(prefix: 'http://localhost:8003');
    test('can be instantiated', () {
      expect(MiddlewareApiUtilities(prefix: ''), isNotNull);
    });

    test('can get modules', () async {
      final modules = await modulesApi.getModules(robotName: robotName);
      expect(modules, isNotEmpty);
    });

    test('can create and delete module', () async {
      final creationResult = await modulesApi.createModule(
        robotName: robotName,
        moduleID: moduleID,
        drawerID: drawerID,
        position: position,
        size: size,
        variant: variant,
      );
      expect(creationResult, isTrue);
      final modules = await modulesApi.getModules(robotName: robotName);
      expect(
        modules.where((drawer) => drawer.moduleID == moduleID && drawer.drawerID == drawerID),
        isNotEmpty,
      );
      final deletionResult = await modulesApi.deleteModule(
        robotName: robotName,
        moduleID: moduleID,
        drawerID: drawerID,
      );
      expect(deletionResult, isTrue);
    });

    test('can fill and empty module', () async {
      final creationResult = await modulesApi.createModule(
        robotName: robotName,
        moduleID: moduleID,
        drawerID: drawerID,
        position: position,
        size: size,
        variant: variant,
      );
      expect(creationResult, isTrue);
      final fillModuleResult = await modulesApi.updateModuleContent(
        robotName: robotName,
        moduleID: moduleID,
        drawerID: drawerID,
        content: {itemID: quantity},
      );
      expect(fillModuleResult, isTrue);
      final modules = await modulesApi.getModules(robotName: robotName);
      final filledModule = modules.firstWhere(
        (drawer) => drawer.moduleID == moduleID && drawer.drawerID == drawerID,
        orElse: () => throw Exception('Drawer not found'),
      );
      expect(filledModule.content, equals({'Stifte': 3}));
      final emptyResult = await modulesApi.emptyModule(
        robotName: robotName,
        moduleID: moduleID,
        drawerID: drawerID,
      );
      expect(emptyResult, isTrue);
      final modules2 = await modulesApi.getModules(robotName: robotName);
      final emptiedModule = modules2.firstWhere(
        (drawer) => drawer.moduleID == moduleID && drawer.drawerID == drawerID,
        orElse: () => throw Exception('Drawer not found'),
      );
      expect(emptiedModule.content, equals({}));
      final deletionResult = await modulesApi.deleteModule(
        robotName: robotName,
        moduleID: moduleID,
        drawerID: drawerID,
      );
      expect(deletionResult, isTrue);
    });

    test('can reserve and free drawer', () async {
      final creationResult = await modulesApi.createModule(
        robotName: robotName,
        moduleID: moduleID,
        drawerID: drawerID,
        position: position,
        size: size,
        variant: variant,
      );
      expect(creationResult, isTrue);
      final reserveResult = await modulesApi.reserveModule(
        robotName: robotName,
        moduleID: moduleID,
        drawerID: drawerID,
        userIDs: ['user1', 'user2'],
        userGroups: ['group1', 'group2'],
      );
      expect(reserveResult, isTrue);
      final modules = await modulesApi.getModules(robotName: robotName);
      final reservedModule = modules.firstWhere(
        (drawer) => drawer.moduleID == moduleID && drawer.drawerID == drawerID,
        orElse: () => throw Exception('Drawer not found'),
      );
      expect(reservedModule.reservedForIds, equals(['user1', 'user2']));
      expect(reservedModule.reservedForGroups, equals(['group1', 'group2']));
      final freeResult = await modulesApi.freeModule(
        robotName: robotName,
        moduleID: moduleID,
        drawerID: drawerID,
      );
      expect(freeResult, isTrue);
      final modules2 = await modulesApi.getModules(robotName: robotName);
      final freedModule = modules2.firstWhere(
        (drawer) => drawer.moduleID == moduleID && drawer.drawerID == drawerID,
        orElse: () => throw Exception('Drawer not found'),
      );
      expect(freedModule.reservedForIds, isEmpty);
      expect(freedModule.reservedForGroups, isEmpty);
      final deletionResult = await modulesApi.deleteModule(
        robotName: robotName,
        moduleID: moduleID,
        drawerID: drawerID,
      );
      expect(deletionResult, isTrue);
    });
  });
}
