import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/controller/delivery_time_controller.dart';
import 'package:robot_frontend/models/controller/location_selection_controller.dart';
import 'package:robot_frontend/models/controller/user_groups_selection_controller.dart';
import 'package:robot_frontend/models/controller/user_selection_controller.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/models/provider/task_provider.dart';
import 'package:robot_frontend/widgets/custom_elevated_button.dart';
import 'package:robot_frontend/widgets/dialogs/nfc_missing_dialog.dart';
import 'package:robot_frontend/widgets/selectors/location_selector.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';
import 'package:robot_frontend/widgets/selectors/time_selector.dart';
import 'package:robot_frontend/widgets/selectors/user_groups_selector.dart';
import 'package:robot_frontend/widgets/selectors/user_selector.dart';

class ContentDistributionView extends StatefulWidget {
  const ContentDistributionView({
    required this.preselectedSubmodules,
    required this.userSelectionControllers,
    required this.userGroupsSelectionControllers,
    required this.locationSelectionControllers,
    required this.deliveryTimeControllers,
    super.key,
  }) : assert(
          preselectedSubmodules.length == userSelectionControllers.length &&
              preselectedSubmodules.length == locationSelectionControllers.length &&
              preselectedSubmodules.length == userGroupsSelectionControllers.length &&
              preselectedSubmodules.length == deliveryTimeControllers.length,
          'preselectedSubmodules, userSelectionControllers, userGroupsSelectionControllers, locationSelectionControllers and deliveryTimeControllers must have the same length',
        );

  final List<SubmoduleAddress> preselectedSubmodules;
  final List<UserSelectionController> userSelectionControllers;
  final List<UserGroupsSelectionController> userGroupsSelectionControllers;
  final List<LocationSelectionController> locationSelectionControllers;
  final List<DeliveryTimeController> deliveryTimeControllers;

  @override
  State<ContentDistributionView> createState() => _ContentDistributionViewState();
}

class _ContentDistributionViewState extends State<ContentDistributionView> {
  bool validateContentToTargetAssignments() {
    for (var i = 0; i < widget.preselectedSubmodules.length; i++) {
      final userSelectionController = widget.userSelectionControllers[i];
      final userGroupsSelectionController = widget.userGroupsSelectionControllers[i];
      if (userSelectionController.selectedUser == null && userGroupsSelectionController.selectionAsStringList().isEmpty) {
        return false;
      }
      final locationController = widget.locationSelectionControllers[i];
      if (locationController.room == null) {
        return false;
      }
    }
    return true;
  }

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.only(top: 32, left: 32, right: 32),
      child: Column(
        children: [
          Expanded(
            child: Selector<ModuleProvider, List<Submodule>>(
              selector: (context, provider) => provider.submodules,
              builder: (context, submodules, child) {
                final selectedSubmodules = submodules
                    .where(
                      (submodule) => widget.preselectedSubmodules.any((preselectedSubmodule) => preselectedSubmodule == submodule.address),
                    )
                    .toList();
                return ListView(
                  children: List.generate(widget.preselectedSubmodules.length, (index) {
                    final submodule = selectedSubmodules[index];
                    return RoundedContainer(
                      child: Padding(
                        padding: const EdgeInsets.all(8),
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Padding(
                              padding: const EdgeInsets.only(left: 8, bottom: 4),
                              child: Text(
                                'Modul ${submodule.address.moduleID} Submodul ${submodule.address.submoduleID}',
                                style: const TextStyle(fontSize: 32, color: RobotColors.primaryText),
                              ),
                            ),
                            RoundedContainer(
                              child: Padding(
                                padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
                                child: Row(
                                  children: [
                                    const Text(
                                      'Inhalte',
                                      style: TextStyle(fontSize: 24, color: RobotColors.secondaryText),
                                    ),
                                    Expanded(
                                      child: Text(
                                        submodule.contentToString(),
                                        textAlign: TextAlign.end,
                                        style: const TextStyle(fontSize: 24, color: RobotColors.secondaryText),
                                      ),
                                    ),
                                  ],
                                ),
                              ),
                            ),
                            const SizedBox(height: 8),
                            Row(
                              children: [
                                Expanded(
                                  flex: 2,
                                  child: LocationSelector(
                                    controller: widget.locationSelectionControllers[index],
                                    label: 'Zielort',
                                    onChanged: () => setState(() {}),
                                  ),
                                ),
                                const SizedBox(
                                  width: 8,
                                ),
                                Expanded(
                                  child: TimeSelector(
                                    deliveryTimeController: widget.deliveryTimeControllers[index],
                                  ),
                                )
                              ],
                            ),
                            const SizedBox(height: 8),
                            Row(
                              children: [
                                Expanded(
                                  child: UserSelector(
                                    controller: widget.userSelectionControllers[index],
                                    onChanged: () => setState(() {}),
                                  ),
                                ),
                                const SizedBox(width: 8),
                                Expanded(
                                  child: UserGroupsSelector(
                                    controller: widget.userGroupsSelectionControllers[index],
                                    onChanged: () => setState(() {}),
                                  ),
                                ),
                              ],
                            ),
                          ],
                        ),
                      ),
                    );
                  }).toList(),
                );
              },
            ),
          ),
          Row(
            mainAxisAlignment: MainAxisAlignment.end,
            children: [
              Padding(
                padding: const EdgeInsets.symmetric(vertical: 32),
                child: CustomElevatedButton(
                  enabled: validateContentToTargetAssignments(),
                  onPressed: () async {
                    final usersWithMissingNfc = widget.userSelectionControllers
                        .map((controller) => controller.selectedUser)
                        .where((user) => user != null && user.nfcID.isEmpty)
                        .toList();
                    if (usersWithMissingNfc.isNotEmpty) {
                      showDialog(
                        context: context,
                        builder: (context) => NfcMissingDialog(
                          identifiers: usersWithMissingNfc.map((user) => '${user!.firstName} ${user.lastName}').toList(),
                        ),
                      );
                      return;
                    }

                    final submodules = Provider.of<ModuleProvider>(context, listen: false).submodules;
                    final taskProvider = Provider.of<TaskProvider>(context, listen: false);
                    for (var i = 0; i < widget.preselectedSubmodules.length; i++) {
                      final submoduleAddress = widget.preselectedSubmodules[i];
                      final submodule = submodules.firstWhere((element) => element.address == submoduleAddress);
                      final controller = widget.userSelectionControllers[i];
                      final user = controller.selectedUser;
                      final userGroups = widget.userGroupsSelectionControllers[i].selectionAsStringList();
                      await taskProvider.createDirectDropoffTask(
                        robotName: 'rb_theron',
                        dropoffTargetID: widget.locationSelectionControllers[i].room!,
                        user: user,
                        userGroups: userGroups,
                        submodule: submodule,
                        earliestStartTime: widget.deliveryTimeControllers[i].timeAsSecondsSinceEpoch(),
                      );
                    }
                    if (context.mounted) {
                      Navigator.pop(context);
                    }
                  },
                  label: 'Fertig',
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }
}
