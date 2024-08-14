import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/controller/user_selection_controller.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';

class UserSelector extends StatefulWidget {
  const UserSelector({
    required this.controller,
    this.initWithSessionUser = false,
    super.key,
  });

  final UserSelectionController controller;
  final bool initWithSessionUser;

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
    return Card(
      color: Colors.white.withOpacity(0.4),
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        child: Row(
          children: [
            const Text(
              'Autorisierte Person',
              textAlign: TextAlign.end,
              style: TextStyle(
                fontSize: 24,
              ),
            ),
            const SizedBox(
              width: 16,
            ),
            Expanded(
              child: FutureBuilder<List<User>>(
                future: initUserSelectorFuture,
                builder: (context, snapshot) {
                  return DropdownButton<User>(
                    disabledHint: const Text(''),
                    isExpanded: true,
                    value: widget.controller.selectedUser,
                    items: snapshot.connectionState != ConnectionState.done
                        ? []
                        : snapshot.data!
                            .map(
                              (e) => DropdownMenuItem<User>(
                                value: e,
                                child: Text(
                                  ' ${e.title}${e.title.isNotEmpty ? ' ' : ''}${e.firstName} ${e.lastName}',
                                  style: const TextStyle(
                                    fontSize: 24,
                                  ),
                                ),
                              ),
                            )
                            .toList(),
                    onChanged: (value) {
                      setState(() {
                        widget.controller.selectedUser = value;
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
              },
              icon: const Icon(Icons.delete),
            ),
          ],
        ),
      ),
    );
  }
}
