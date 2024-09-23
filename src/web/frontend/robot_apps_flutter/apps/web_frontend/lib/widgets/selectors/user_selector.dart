import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:web_frontend/constants/web_colors.dart';
import 'package:web_frontend/models/provider/user_provider.dart';
import 'package:web_frontend/widgets/rounded_container.dart';

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
      widget.controller.selectedUser = userProvider.user;
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
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Autorisierte Person',
              textAlign: TextAlign.end,
              style: TextStyle(
                fontSize: 24,
                color: WebColors.secondaryText,
              ),
            ),
            Row(
              children: [
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
                                        fontSize: 18,
                                        color: WebColors.secondaryText,
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
          ],
        ),
      ),
    );
  }
}
