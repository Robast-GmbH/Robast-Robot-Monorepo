import 'dart:async';

import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/controller/location_selection_controller.dart';
import 'package:robot_frontend/models/controller/user_groups_selection_controller.dart';
import 'package:robot_frontend/models/controller/user_name_controller.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/widgets/location_selector.dart';
import 'package:robot_frontend/widgets/nfc_writing_dialog.dart';
import 'package:robot_frontend/widgets/user_groups_selector.dart';
import 'package:robot_frontend/widgets/user_name_editor.dart';
import 'package:uuid/uuid.dart';

class UserManagementListTile extends StatefulWidget {
  const UserManagementListTile({required this.user, required this.onUserUpdate, super.key});
  final User user;
  final VoidCallback onUserUpdate;
  @override
  State<UserManagementListTile> createState() => _UserManagementListTileState();
}

class _UserManagementListTileState extends State<UserManagementListTile> {
  final userNameController = UserNameController();
  final userGroupsSelectionController = UserGroupsSelectionController();
  final locationSelectionController = LocationSelectionController();

  @override
  void initState() {
    super.initState();

    userNameController
      ..title = widget.user.title
      ..firstName = widget.user.firstName
      ..lastName = widget.user.lastName;

    userGroupsSelectionController
      ..isPatient = widget.user.userGroups.contains('PATIENT')
      ..isStaff = widget.user.userGroups.contains('STAFF')
      ..isAdmin = widget.user.userGroups.contains('ADMIN');

    locationSelectionController
      ..setStation(widget.user.station == '' ? null : widget.user.station)
      ..setRoom(widget.user.room == '' ? null : widget.user.room);
  }

  @override
  Widget build(BuildContext context) {
    return Card(
      color: Colors.white.withOpacity(0.5),
      child: Padding(
        padding: const EdgeInsets.all(8),
        child: Column(
          children: [
            Row(
              children: [
                Expanded(
                  child: UserNameEditor(
                    controller: userNameController,
                  ),
                ),
                const SizedBox(
                  width: 8,
                ),
                Expanded(
                  child: UserGroupsSelector(
                    controller: userGroupsSelectionController,
                  ),
                ),
                const SizedBox(
                  width: 8,
                ),
                IconButton(
                  iconSize: 32,
                  color: Colors.white,
                  onPressed: () {
                    final userGroups = userGroupsSelectionController.selectionAsStringList();
                    Provider.of<UserProvider>(context, listen: false).updateUser(
                      updatedUser: User(
                        id: widget.user.id,
                        nfcID: widget.user.nfcID,
                        title: userNameController.title,
                        firstName: userNameController.firstName,
                        lastName: userNameController.lastName,
                        station: locationSelectionController.station ?? '',
                        room: locationSelectionController.room ?? '',
                        userGroups: userGroups,
                      ),
                    );
                  },
                  icon: const Icon(Icons.save_alt),
                ),
              ],
            ),
            const SizedBox(
              height: 8,
            ),
            Row(
              children: [
                Expanded(
                  child: LocationSelector(
                    controller: locationSelectionController,
                    label: 'Standort',
                  ),
                ),
                const SizedBox(
                  width: 8,
                ),
                PopupMenuButton(
                  iconSize: 32,
                  itemBuilder: (BuildContext context) => <PopupMenuEntry<int>>[
                    PopupMenuItem(
                      child: ListTile(
                        leading: const Icon(Icons.nfc),
                        title: const Text('NFC beschreiben'),
                        onTap: () async {
                          Navigator.pop(context);
                          final newNfcID = const Uuid().v4();
                          unawaited(
                            Provider.of<UserProvider>(context, listen: false).updateUser(
                              updatedUser: User(
                                id: widget.user.id,
                                nfcID: newNfcID,
                                title: widget.user.title,
                                firstName: widget.user.firstName,
                                lastName: widget.user.lastName,
                                station: widget.user.station,
                                room: widget.user.room,
                                userGroups: widget.user.userGroups,
                              ),
                            ),
                          );
                          await showDialog<NFCWritingDialog>(
                            context: context,
                            builder: (context) => NFCWritingDialog(
                              nfcData: newNfcID,
                            ),
                          );
                        },
                      ),
                    ),
                    PopupMenuItem(
                      child: ListTile(
                        leading: const Icon(Icons.delete),
                        title: const Text('Löschen'),
                        onTap: () async {
                          Navigator.pop(context);
                          await Provider.of<UserProvider>(context, listen: false).deleteUser(id: widget.user.id);
                          widget.onUserUpdate();
                        },
                      ),
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}
