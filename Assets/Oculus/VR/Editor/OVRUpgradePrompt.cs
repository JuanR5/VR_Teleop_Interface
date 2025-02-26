/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * Licensed under the Oculus SDK License Agreement (the "License");
 * you may not use the Oculus SDK except in compliance with the License,
 * which is provided at the time of installation or download, or which
 * otherwise accompanies this software in either electronic or hard copy form.
 *
 * You may obtain a copy of the License at
 *
 * https://developer.oculus.com/licenses/oculussdk/
 *
 * Unless required by applicable law or agreed to in writing, the Oculus SDK
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

[InitializeOnLoad]
public class OVRUpgradePrompt : EditorWindow
{
    private static readonly Vector2 s_windowSize = new Vector2(500, 320);

    private const string SamplesURL = "https://developer.oculus.com/documentation/unity/unity-import-samples/";
    private const string DocumentationURL =
        "https://developer.oculus.com/documentation/unity/unity-package-manager/";
    private const string MigrationGuideURL = "https://developer.oculus.com/documentation/unity/unity-package-manager/#migrate-from-oculus-integration-sdk";
    private const string UPMListURL = "https://assetstore.unity.com/lists/meta-sdks-9071889420297";
    private const string FeedbackURL = "https://communityforums.atmeta.com/t5/Unity-VR-Development/bd-p/dev-unity";
    private const string AllInOneLink = "https://assetstore.unity.com/packages/tools/integration/meta-xr-all-in-one-sdk-269657";

    private static readonly string _hasShownOnceKey = "OVRUpgradePrompt_hasShownOnce";

    private static readonly StyleColor s_linkColor =
        new StyleColor(new Color32(29, 101, 193, 255));
    private static readonly StyleColor s_linkHoverColor =
        new StyleColor(new Color32(44, 132, 193, 255));

    static OVRUpgradePrompt()
    {
        EditorApplication.delayCall += DelayCall;
    }

    private static void DelayCall()
    {
        var projectConfig = OVRProjectConfig.CachedProjectConfig;
        if (!SessionState.GetBool(_hasShownOnceKey, false) && projectConfig.showUPMUpdatePrompt)
        {
            ShowWindow();
            SessionState.SetBool(_hasShownOnceKey, true);
        }
        EditorApplication.delayCall -= DelayCall;
    }

    [MenuItem("Oculus/Update Available!")]
    public static void ShowWindow()
    {
        var window =
            GetWindow<OVRUpgradePrompt>(
                title: "Meta XR UPM Notice",
                focus: true);
        window.minSize = s_windowSize;
        window.maxSize = s_windowSize;
        window.Show();
    }

    private void CreateGUI()
    {
        var container = new VisualElement
        {
            style =
                {
                    paddingLeft = 8,
                    paddingTop = 8,
                    paddingRight = 8,
                    paddingBottom = 8,
                    flexDirection = FlexDirection.Column,
                    justifyContent = Justify.FlexStart,
                    alignItems = Align.Stretch,
                    flexGrow = 1f,
                }
        };

        var label = new Label
        {
            text = "The \"Oculus Integrations\" package is deprecated, and no longer receiving updates. Meta XR SDKs are now distributed as Unity Package Manager (UPM) packages, which provide a more flexible and efficient integration process." +
            "\n\nDownload our new UPM releases to get the latest Meta XR features. The easiest way to get started is via the All-in-One SDK, which provides a similar set of features as the legacy Oculus Integrations package, and new features including:" +
            "\n\n  • Depth API, which can be used to render occlusions \n  • Inside-out body tracking and generative legs \n  • Multimodal input support (hands and controllers simultaneously) " +
            "\n  • IL2CPP link time optimizations \n  • New SDKs like Haptics and Mixed Reality Utility Kit",
            style =
                {
                    whiteSpace = WhiteSpace.Normal
                }
        };
        container.Add(label);

        var samplesContainer = new VisualElement
        {
            style =
                {
                    marginTop = 12,
                    marginBottom = 0,
                    flexDirection = FlexDirection.Row,
                    alignItems = Align.Center,
                    justifyContent = Justify.FlexStart
                }
        };
        var samplesLabel = new Label
        {
            text = "Note that sample assets are now imported separately, see",
        };
        var samplesLink = CreateHyperlinkButton("instructions here.", SamplesURL);
        samplesContainer.Add(samplesLabel);
        samplesContainer.Add(samplesLink);
        container.Add(samplesContainer);

        var docsContainer = new VisualElement
        {
            style =
                {
                    marginTop = 1,
                    marginBottom = 4,
                    flexDirection = FlexDirection.Row,
                    alignItems = Align.Center,
                    justifyContent = Justify.FlexStart
                }
        };
        var docsLabel = new Label
        {
            text = "For more information, see the ",
        };
        var docsLink = CreateHyperlinkButton("UPM documentation", DocumentationURL);
        var migrationLabel = new Label
        {
            text = " and ",
        };
        var migrationLink = CreateHyperlinkButton("migration guide.", MigrationGuideURL);
        docsContainer.Add(docsLabel);
        docsContainer.Add(docsLink);
        docsContainer.Add(migrationLabel);
        docsContainer.Add(migrationLink);
        container.Add(docsContainer);


        var _allInOneButton = new Button(OnOpenClick)
        {
            text = "Open Meta XR All-in-One SDK (UPM) on Asset Store",
            style =
                {
                    paddingLeft = 8,
                    paddingTop = 8,
                    paddingRight = 8,
                    paddingBottom = 8,
                    marginTop = 4,
                    fontSize = 14,
                    display = DisplayStyle.Flex,
                }
        };
        container.Add(_allInOneButton);

        var footer = new VisualElement
        {
            style =
                {
                    height = 20,
                    marginTop = 6,
                    flexDirection = FlexDirection.Row,
                    alignContent = Align.Stretch,
                }
        };
        var footerLeft = new VisualElement
        {
            style =
                {
                    flexDirection = FlexDirection.Row
                }
        };
        var footerRight = new VisualElement
        {
            style =
                {
                    flexDirection = FlexDirection.Row, marginLeft = StyleKeyword.Auto,
                }
        };
        footer.Add(footerLeft);
        footer.Add(footerRight);

        var _disablePromptCheckbox = new Toggle
        {
            style =
                {
                    marginBottom = 0,
                },
            text = " Do not show again",
        };
        var projectConfig = OVRProjectConfig.CachedProjectConfig;
        _disablePromptCheckbox.value = !projectConfig.showUPMUpdatePrompt;
        _disablePromptCheckbox.RegisterValueChangedCallback(evt =>
        {
            projectConfig.showUPMUpdatePrompt = !_disablePromptCheckbox.value;
            OVRProjectConfig.CommitProjectConfig(projectConfig);
        });
        footerLeft.Add(_disablePromptCheckbox);

        var footerSamplesLink = CreateHyperlinkButton("All Meta UPM Packages", UPMListURL);
        footerSamplesLink.style.height = 20;
        footerRight.Add(footerSamplesLink);

        var delimiter = new TextElement
        {
            text = "|",
            style =
                {
                    width = 12, height = 20, unityTextAlign = TextAnchor.MiddleCenter,
                }
        };
        footerRight.Add(delimiter);

        var footerDocsLink = CreateHyperlinkButton("Feedback", FeedbackURL);
        footerDocsLink.style.height = 20;
        footerRight.Add(footerDocsLink);

        container.Add(footer);
        rootVisualElement.Add(container);
    }

    private static Button CreateHyperlinkButton(string label, string url)
    {
        var button = new Button(() => Application.OpenURL(url))
        {
            text = label,
            style =
                {
                    color = s_linkColor,
                    backgroundImage = null,
                    backgroundColor = Color.clear,
                    borderTopColor = Color.clear,
                    borderRightColor = Color.clear,
                    borderBottomColor = Color.clear,
                    borderLeftColor = Color.clear,
                    paddingLeft = 0,
                    paddingRight = 0,
                    paddingTop = 0,
                    paddingBottom = 0,
                    marginLeft = 0,
                    marginRight = 0,
                    marginTop = 0,
                    marginBottom = 0,
                    borderTopWidth = 0,
                    borderBottomWidth = 0,
                },
            tooltip = url
        };
        button.RegisterCallback((MouseOverEvent _) =>
        {
            button.style.color = s_linkHoverColor;
        });
        button.RegisterCallback((MouseOutEvent _) =>
        {
            button.style.color = s_linkColor;
        });
        return button;
    }

    private static void OnOpenClick()
    {
        Application.OpenURL(AllInOneLink);
    }
}
