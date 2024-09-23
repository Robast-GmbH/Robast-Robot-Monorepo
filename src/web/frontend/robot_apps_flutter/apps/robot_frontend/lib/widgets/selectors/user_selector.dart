import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';
import 'package:shared_data_models/shared_data_models.dart';

class UserSelector extends StatefulWidget {
  const UserSelector({
    required this.controller,
    this.initWithSessionUser = false,
    this.onChanged,
    super.key,
  });

  final UserSelectionController controller;
  final bool initWithSessionUser;
  final void Function()? onChanged;

  @override
  State<UserSelector> createState() => _UserSelectorState();
}

class _UserSelectorState extends State<UserSelector> {
  late Future<List<User>> initUserSelectorFuture;

  Future<List<User>> initUserSelector() async {
    final userProvider = Provider.of<UserProvider>(context, listen: false);
    final users = await userProvider.getUsers();
    if (widget.initWithSessionUser) {
      final userSession = await userProvider.getUserSession(robotName: 'rb_theron');
      if (userSession != null) {
        widget.controller.selectedUser = users.firstWhere((element) => userSession.id == element.id);
      }
      widget.onChanged?.call();
    }
    return users;
  }

  @override
  void initState() {
    super.initState();
    initUserSelectorFuture = initUserSelector();
  }

  @override
  Widget build(BuildContext context) {
    return RoundedContainer(
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        child: Row(
          children: [
            const Text(
              'Autorisierte Person',
              textAlign: TextAlign.end,
              style: TextStyle(
                fontSize: 24,
                color: RobotColors.secondaryText,
              ),
            ),
            const SizedBox(
              width: 16,
            ),
            Expanded(
              child: FutureBuilder<List<User>>(
                future: initUserSelectorFuture,
                builder: (context, snapshot) {
                  return DropdownButtonFormField<String>(
                    isDense: false,
                    decoration: const InputDecoration(contentPadding: EdgeInsets.zero),
                    disabledHint: const Text(''),
                    isExpanded: true,
                    value: widget.controller.selectedUser?.id,
                    items: snapshot.connectionState != ConnectionState.done
                        ? []
                        : snapshot.data!
                            .map(
                              (e) => DropdownMenuItem<String>(
                                value: e.id,
                                child: Text(
                                  ' ${e.title}${e.title.isNotEmpty ? ' ' : ''}${e.firstName} ${e.lastName}',
                                  style: const TextStyle(
                                    fontSize: 24,
                                    color: RobotColors.secondaryText,
                                  ),
                                ),
                              ),
                            )
                            .toList(),
                    onChanged: (value) {
                      setState(() {
                        if (snapshot.data?.any((element) => element.id == value) ?? false) {
                          widget.controller.selectedUser = snapshot.data!.firstWhere((element) => element.id == value);
                        }
                        widget.onChanged?.call();
                      });
                    },
                  );
                },
              ),
            ),
            const SizedBox(
              width: 8,
            ),
            IconButton(
              onPressed: () {
                setState(() {
                  widget.controller.selectedUser = null;
                });
                widget.onChanged?.call();
              },
              icon: const Icon(Icons.delete),
            ),
          ],
        ),
      ),
    );
  }
}
