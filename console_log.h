// Copyright (C) 2023-2026 Ola Gatner
// SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
#pragma once
// Captures everything AmyCore prints (cout, printf, stderr) into a line buffer for the ImGui "Console"
// window, and still forwards it to the original stdout (terminal or log file).
// stdout/stderr are redirected into a pipe; a reader thread copies the pipe to the original stdout and
// splits it into lines.

#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>
#include <stdio.h>
#include "imgui.h"

struct ConsoleLog
{
    std::mutex mtx;
    std::deque<std::string> lines;       // everything except frame dumps
    std::deque<std::string> frames;      // "framerec type:.." dumps (thousands per second), kept separately
    size_t maxFrames=500;
    std::string partial;
    size_t maxLines=5000;
    unsigned long long dropped=0;     // lines removed because of maxLines
    bool paused=false;                // GUI: stop taking new lines (they still go to stdout)

    void addText(const char *buf, size_t n)
    {
        std::lock_guard<std::mutex> lock(mtx);
        for (size_t i=0; i<n; i++)
        {
            char c=buf[i];
            if (c=='\r') continue;
            if (c=='\n')
            {
                if (!paused)
                {
                    if (partial.find("framerec type:")!=std::string::npos)
                    {
                        frames.push_back(partial);
                        while (frames.size()>maxFrames) frames.pop_front();
                    }
                    else
                    {
                        lines.push_back(partial);
                        while (lines.size()>maxLines) { lines.pop_front(); dropped++; }
                    }
                }
                partial.clear();
            }
            else if (partial.size()<1000) partial+=c;
        }
    }

    // redirect stdout and stderr into this log; call once, early in main()
    void startCapture()
    {
        int p[2];
        if (pipe(p)!=0) return;
        fflush(stdout); fflush(stderr);
        int origOut=dup(STDOUT_FILENO);
        dup2(p[1], STDOUT_FILENO);
        dup2(p[1], STDERR_FILENO);
        close(p[1]);
        setvbuf(stdout, nullptr, _IOLBF, 0);         // line buffered, so printf lines show up at once
        std::thread([this, rd=p[0], origOut]()
        {
            char buf[4096];
            ssize_t n;
            while ((n=read(rd, buf, sizeof buf))>0)
            {
                ssize_t off=0;
                while (off<n) { ssize_t w=write(origOut, buf+off, n-off); if (w<=0) break; off+=w; }
                addText(buf, (size_t)n);
            }
        }).detach();
    }

    void draw(bool *open)
    {
        static ImGuiTextFilter filter;
        static bool autoscroll=true;
        static bool showFrames=false;                 // show the last "framerec type:.." dumps instead of the log
        ImGui::SetNextWindowSize(ImVec2(900,400), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowBgAlpha(1.0f);
        if (!ImGui::Begin("Console", open)) { ImGui::End(); return; }
        if (ImGui::Button("Clear")) { std::lock_guard<std::mutex> lock(mtx); lines.clear(); frames.clear(); }
        ImGui::SameLine(); ImGui::Checkbox("Pause", &paused);
        ImGui::SameLine(); ImGui::Checkbox("Autoscroll", &autoscroll);
        ImGui::SameLine(); ImGui::Checkbox("show frame dumps", &showFrames);
        ImGui::SameLine(); filter.Draw("filter", 200);
        ImGui::Separator();
        ImGui::BeginChild("scroll", ImVec2(0,0), false, ImGuiWindowFlags_HorizontalScrollbar);
        std::vector<const std::string*> shown;
        {
            std::lock_guard<std::mutex> lock(mtx);
            auto &src = showFrames ? frames : lines;
            shown.reserve(src.size());
            for (auto &l:src)
            {
                if (filter.IsActive() && !filter.PassFilter(l.c_str())) continue;
                shown.push_back(&l);
            }
            ImGuiListClipper clipper;
            clipper.Begin((int)shown.size());
            while (clipper.Step())
                for (int i=clipper.DisplayStart; i<clipper.DisplayEnd; i++)
                {
                    const std::string &l=*shown[i];
                    bool err = l.find("ERROR")!=std::string::npos || l.find("FAILED")!=std::string::npos;
                    if (err) ImGui::TextColored(ImVec4(1,0.4f,0.4f,1), "%s", l.c_str());
                    else     ImGui::TextUnformatted(l.c_str());
                }
        }
        if (autoscroll && !paused && ImGui::GetScrollY()>=ImGui::GetScrollMaxY()-20) ImGui::SetScrollHereY(1.0f);
        ImGui::EndChild();
        ImGui::End();
    }
};
