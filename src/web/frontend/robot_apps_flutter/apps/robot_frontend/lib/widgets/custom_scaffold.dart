import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/custom_focus_node.dart';
import 'package:robot_frontend/models/provider/keyboard_provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/pages/manuals_page.dart';
import 'package:robot_frontend/widgets/background_view.dart';
import 'package:robot_frontend/widgets/clock_view.dart';
import 'package:robot_frontend/widgets/emergency_stop_view.dart';
import 'package:robot_frontend/widgets/status_indicator_view.dart';
import 'package:robot_frontend/widgets/titled_view.dart';
import 'package:virtual_keyboard_custom_layout/virtual_keyboard_custom_layout.dart';

class CustomScaffold extends StatelessWidget {
  const CustomScaffold({
    super.key,
    this.child = const SizedBox(),
    this.showBackButton = true,
    this.inactivityTimerEnabled = true,
    this.onBackButtonPressed,
    this.title = '',
    this.ignoreMissingEmergencyStopData = false,
    this.collapsedTitle = false,
  });

  final Widget child;
  final bool showBackButton;
  final bool inactivityTimerEnabled;
  final VoidCallback? onBackButtonPressed;
  final String title;
  final bool ignoreMissingEmergencyStopData;
  final bool collapsedTitle;

  @override
  Widget build(BuildContext context) {
    print('rebuild CustomScaffold');
    return Scaffold(
      body: BackgroundView(
        inactivityTimerEnabled: inactivityTimerEnabled,
        child: Selector<RobotProvider, bool?>(
          selector: (context, robotProvider) => robotProvider.isEmergencyStopPressed,
          builder: (context, isEmergencyStopPressed, selectorChild) {
            if (isEmergencyStopPressed == null && !ignoreMissingEmergencyStopData) {
              return Column(
                children: [
                  Row(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Expanded(
                        child: Row(
                          mainAxisAlignment: MainAxisAlignment.start,
                          children: [
                            Padding(
                              padding: const EdgeInsets.only(left: 16, top: 8),
                              child: IconButton(
                                padding: const EdgeInsets.all(0),
                                onPressed: () async {
                                  final robotProvider = Provider.of<RobotProvider>(context, listen: false);
                                  robotProvider.blockNavigation();
                                  await Navigator.push(context, MaterialPageRoute<ManualsPage>(builder: (context) => const ManualsPage()));
                                  robotProvider.unblockNavigation();
                                },
                                color: RobotColors.primaryIcon,
                                icon: const Icon(
                                  Icons.info_outline,
                                  size: 64,
                                ),
                              ),
                            ),
                          ],
                        ),
                      ),
                      const Padding(
                        padding: EdgeInsets.all(8),
                        child: ClockView(),
                      ),
                      const Expanded(
                        child: Padding(
                          padding: EdgeInsets.only(top: 8, right: 16),
                          child: StatusIndicatorView(
                            shouldBlockNavigation: true,
                          ),
                        ),
                      ),
                    ],
                  ),
                  const Expanded(
                    child: Center(
                        child: Padding(
                      padding: EdgeInsets.symmetric(horizontal: 256),
                      child: Text(
                        "Not-Aus Status konnte nicht abgerufen werden",
                        style: TextStyle(color: RobotColors.primaryText, fontSize: 72),
                        textAlign: TextAlign.center,
                      ),
                    )),
                  ),
                ],
              );
            }
            if (isEmergencyStopPressed ?? false) {
              Provider.of<KeyboardProvider>(context, listen: false).unfocus();
              Navigator.of(context).popUntil((route) => route is PageRoute);
              return const EmergencyStopView();
            }
            return Stack(
              children: [
                GestureDetector(
                  onTap: () => Provider.of<KeyboardProvider>(context, listen: false).unfocus(),
                  child: Container(
                    width: double.infinity,
                    height: double.infinity,
                    color: Colors.transparent,
                    child: TitledView(
                      title: title,
                      showBackButton: showBackButton,
                      onBackButtonPressed: onBackButtonPressed,
                      collapsedTitle: collapsedTitle,
                      child: child,
                    ),
                  ),
                ),
                Selector<KeyboardProvider, CustomFocusNode?>(
                    selector: (context, provider) => provider.focusNode,
                    builder: (context, focusNode, child) {
                      if (focusNode == null) {
                        return const SizedBox();
                      }
                      bool shiftPressed = false;
                      return Align(
                        alignment: Alignment.bottomCenter,
                        child: ColoredBox(
                          color: Colors.white,
                          child: VirtualKeyboard(
                            fontSize: 32,
                            height: 400,
                            borderColor: Colors.grey,
                            defaultLayouts: const [VirtualKeyboardDefaultLayouts.German],
                            keys: focusNode.layout == VirtualKeyboardDefaultLayouts.Numeric
                                ? [
                                    ['1', '2', '3'],
                                    ['4', '5', '6'],
                                    ['7', '8', '9'],
                                    ['BACKSPACE', '0', 'RETURN']
                                  ]
                                : null,
                            textController: TextEditingController(text: focusNode.text),
                            onKeyPress: (key) {
                              if (key.keyType == VirtualKeyboardKeyType.Action) {
                                switch (key.action) {
                                  case VirtualKeyboardKeyAction.Backspace:
                                    if (focusNode.text.isNotEmpty) {
                                      focusNode.text = focusNode.text.substring(0, focusNode.text.length - 1);
                                    }
                                    break;
                                  case VirtualKeyboardKeyAction.Return:
                                    if (focusNode.onSubmit != null) {
                                      focusNode.onSubmit?.call();
                                    } else {
                                      Provider.of<KeyboardProvider>(context, listen: false).focusNext();
                                    }

                                    return;
                                  case VirtualKeyboardKeyAction.Space:
                                    focusNode.text = '${focusNode.text} ';
                                    break;
                                  case VirtualKeyboardKeyAction.Shift:
                                    shiftPressed = !shiftPressed;
                                    break;
                                  default:
                                    break;
                                }
                              } else {
                                focusNode.text = focusNode.text + (shiftPressed ? key.capsText! : key.text!);
                              }
                              focusNode.setTextState?.call();
                            },
                          ),
                        ),
                      );
                    }),
              ],
            );
          },
        ),
      ),
    );
  }
}
