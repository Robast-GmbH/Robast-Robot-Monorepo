// ignore_for_file: prefer_const_constructors
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:middleware_api_utilities/src/sub_apis/modules_api.dart';
import 'package:middleware_api_utilities/src/sub_apis/users_api.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:test/test.dart';

void main() {
  group('MiddlewareApiUtilities', () {
    const robotName = 'rb_theron';
    const submoduleAddress = SubmoduleAddress(moduleID: 7, submoduleID: 0);
    const position = 9;
    const size = 1;
    const itemID = 'Stifte';
    const quantity = 3;
    const variant = 'manual';
    final modulesApi = ModulesApi(prefix: 'http://localhost:8003');

    const userID = '';
    const nfcID = '';
    const mail = 'test@test.de';
    const title = 'Prof. Dr. med.';
    const firstName = 'Max';
    const lastName = 'Mustermann';
    const station = 'HNO';
    const room = 'Raum 3';
    const userGroups = ['group1', 'group2'];
    final testUser = User(
      id: userID,
      nfcID: nfcID,
      mail: mail,
      title: title,
      firstName: firstName,
      lastName: lastName,
      station: station,
      room: room,
      userGroups: userGroups,
    );
    const updatedMail = 'update@test.de';
    const updatedTitle = 'Dr. med.';
    const updatedFirstName = 'Maximilian';
    const updatedLastName = 'Musterfrau';
    const updatedStation = 'Orthopädie';
    const updatedRoom = 'Raum 4';
    const updatedUserGroups = ['group3', 'group4'];

    final usersApi = UsersApi(prefix: 'http://localhost:8003');
    test('can be instantiated', () {
      expect(MiddlewareApiUtilities(), isNotNull);
    });

    test('can get modules', () async {
      final modules = await modulesApi.getSubmodules(robotName: robotName);
      expect(modules, isNotEmpty);
    });

    test('can create and delete module', () async {
      final creationResult = await modulesApi.createSubmodule(
        robotName: robotName,
        submoduleAddress: submoduleAddress,
        position: position,
        size: size,
        variant: variant,
      );
      expect(creationResult, isTrue);
      final modules = await modulesApi.getSubmodules(robotName: robotName);
      expect(
        modules.where((submodule) => submodule.address == submoduleAddress),
        isNotEmpty,
      );
      final deletionResult = await modulesApi.deleteSubmodule(robotName: robotName, submoduleAddress: submoduleAddress);
      expect(deletionResult, isTrue);
    });

    test('can fill and empty module', () async {
      final creationResult = await modulesApi.createSubmodule(
        robotName: robotName,
        submoduleAddress: submoduleAddress,
        position: position,
        size: size,
        variant: variant,
      );
      expect(creationResult, isTrue);
      final fillModuleResult = await modulesApi.updateSubmoduleContent(
        robotName: robotName,
        submoduleAddress: submoduleAddress,
        itemsByCount: {itemID: quantity},
      );
      expect(fillModuleResult, isTrue);
      final modules = await modulesApi.getSubmodules(robotName: robotName);
      final filledModule = modules.firstWhere(
        (submodule) => submodule.address == submoduleAddress,
        orElse: () => throw Exception('Submodule not found'),
      );
      expect(filledModule.itemsByCount, equals({'Stifte': 3}));
      final emptyResult = await modulesApi.emptySubmodule(
        robotName: robotName,
        submoduleAddress: submoduleAddress,
      );
      expect(emptyResult, isTrue);
      final modules2 = await modulesApi.getSubmodules(robotName: robotName);
      final emptiedModule = modules2.firstWhere(
        (submodule) => submodule.address == submoduleAddress,
        orElse: () => throw Exception('Submodule not found'),
      );
      expect(emptiedModule.itemsByCount, equals({}));
      final deletionResult = await modulesApi.deleteSubmodule(
        robotName: robotName,
        submoduleAddress: submoduleAddress,
      );
      expect(deletionResult, isTrue);
    });

    test('can reserve and free submodule', () async {
      final creationResult = await modulesApi.createSubmodule(
        robotName: robotName,
        submoduleAddress: submoduleAddress,
        position: position,
        size: size,
        variant: variant,
      );
      expect(creationResult, isTrue);
      final reserveResult = await modulesApi.reserveSubmodule(
        robotName: robotName,
        submoduleAddress: submoduleAddress,
        taskID: 'task1',
        userIDs: ['user1', 'user2'],
        userGroups: ['group1', 'group2'],
      );
      expect(reserveResult, isTrue);
      final modules = await modulesApi.getSubmodules(robotName: robotName);
      final reservedModule = modules.firstWhere(
        (submodule) => submodule.address == submoduleAddress,
        orElse: () => throw Exception('Submodule not found'),
      );
      expect(reservedModule.reservedForTask, equals('task1'));
      expect(reservedModule.reservedForIds, equals(['user1', 'user2']));
      expect(reservedModule.reservedForGroups, equals(['group1', 'group2']));
      final freeResult = await modulesApi.freeSubmodule(
        robotName: robotName,
        submoduleAddress: submoduleAddress,
      );
      expect(freeResult, isTrue);
      final modules2 = await modulesApi.getSubmodules(robotName: robotName);
      final freedModule = modules2.firstWhere(
        (submodule) => submodule.address == submoduleAddress,
        orElse: () => throw Exception('Submodule not found'),
      );
      expect(freedModule.reservedForIds, isEmpty);
      expect(freedModule.reservedForGroups, isEmpty);
      final deletionResult = await modulesApi.deleteSubmodule(
        robotName: robotName,
        submoduleAddress: submoduleAddress,
      );
      expect(deletionResult, isTrue);
    });

    test('can create and delete user', () async {
      final creationResult = await usersApi.createUser(
        newUser: testUser,
      );
      expect(creationResult, isNotNull);
      final users = await usersApi.getUsers();
      expect(
        users.where((user) => user.id == creationResult!.id),
        isNotEmpty,
      );
      final deletionResult = await usersApi.deleteUser(id: creationResult!.id);
      expect(deletionResult, isTrue);
    });

    test('can update user', () async {
      final creationResult = await usersApi.createUser(
        newUser: testUser,
      );
      expect(creationResult, isNotNull);
      final updatedUser = User(
        id: creationResult!.id,
        nfcID: creationResult.nfcID,
        mail: updatedMail,
        title: updatedTitle,
        firstName: updatedFirstName,
        lastName: updatedLastName,
        station: updatedStation,
        room: updatedRoom,
        userGroups: updatedUserGroups,
      );
      final updatedUserResult = await usersApi.updateUser(updatedUser: updatedUser);
      expect(updatedUserResult, isNotNull);
      final user = await usersApi.getUser(id: updatedUserResult!.id);
      expect(user, isNotNull);
      expect(user!.title, equals(updatedTitle));
      expect(user.firstName, equals(updatedFirstName));
      expect(user.lastName, equals(updatedLastName));
      expect(user.station, equals(updatedStation));
      expect(user.room, equals(updatedRoom));
      expect(user.userGroups, equals(updatedUserGroups));
      final deletionResult = await usersApi.deleteUser(id: creationResult.id);
      expect(deletionResult, isTrue);
    });

    // test('can read nfc', () async {
    //   final nfc = NFCApi(prefix: 'http://localhost:8003');
    //   final result = await nfc.readNFC(robotName: robotName);
    //   expect(result, isNotNull);
    // });
  });
}
