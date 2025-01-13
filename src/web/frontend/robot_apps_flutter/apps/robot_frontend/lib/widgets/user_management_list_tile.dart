import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/keyboard_provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/widgets/selectors/location_selector.dart';
import 'package:robot_frontend/widgets/dialogs/nfc_assignment_dialog.dart';
import 'package:robot_frontend/widgets/selectors/user_groups_selector.dart';
import 'package:robot_frontend/widgets/user_name_editor.dart';
import 'package:shared_data_models/shared_data_models.dart';

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

    userGroupsSelectionController.userGroups.addAll(widget.user.userGroups);

    locationSelectionController
      ..setStation(widget.user.station == '' ? null : widget.user.station)
      ..setRoom(widget.user.room == '' ? null : widget.user.room);
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      margin: const EdgeInsets.all(8),
      decoration: BoxDecoration(
        color: Colors.black.withOpacity(0.1),
        borderRadius: BorderRadius.circular(16),
      ),
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
                  color: RobotColors.primaryIcon,
                  onPressed: () {
                    Provider.of<KeyboardProvider>(context, listen: false).unfocus();
                    final userGroups = userGroupsSelectionController.userGroups;
                    Provider.of<UserProvider>(context, listen: false).updateUser(
                      updatedUser: User(
                        id: widget.user.id,
                        nfcID: widget.user.nfcID,
                        mail: widget.user.mail,
                        title: userNameController.title,
                        firstName: userNameController.firstName,
                        lastName: userNameController.lastName,
                        station: locationSelectionController.station ?? '',
                        room: locationSelectionController.room ?? '',
                        userGroups: userGroups,
                      ),
                    );
                  },
                  icon: const Icon(
                    Icons.save_alt,
                    color: RobotColors.secondaryIcon,
                  ),
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
                  iconColor: RobotColors.secondaryIcon,
                  itemBuilder: (BuildContext context) => <PopupMenuEntry<int>>[
                    PopupMenuItem(
                      child: ListTile(
                        leading: const Icon(Icons.nfc),
                        title: const Text('NFC zuweisen'),
                        onTap: () async {
                          Navigator.pop(context);
                          await showDialog<NFCAssignmentDialog>(
                            context: context,
                            builder: (context) => NFCAssignmentDialog(
                              userID: widget.user.id,
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
