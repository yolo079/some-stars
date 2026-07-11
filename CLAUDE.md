# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a GitHub stars list aggregator that automatically generates a categorized README.md from a user's starred repositories. The project consists of:

1. **Generator tool** (`tools/mawesome/index.js`) - Node.js CLI that fetches starred repos via GitHub API and renders an EJS template
2. **Template** (`template/README.ejs`) - EJS template for generating the README
3. **Cloudflare Worker** (`worker.js`) - Serves an interactive HTML page for browsing starred repos
4. **Static data** (`data.json`) - Cached star information (used by the worker)

## Common Commands

### Generate README from GitHub stars

```bash
# From project root
node ./tools/mawesome/index.js \
  --template ./template/README.ejs \
  --github-name "<username>" \
  --out README.md
```

Requires `PERSONAL_TOKEN` or `GITHUB_TOKEN` environment variable for GitHub API authentication.

### Install dependencies

```bash
npm ci --prefix ./tools/mawesome
```

## Architecture

### GitHub Actions Workflow

The workflow (`.github/workflows/star-list.yml`) runs automatically:
- Every 2 days at 07:00 UTC (`cron: '0 7 */2 * *'`)
- On push to `main` branch
- On manual workflow dispatch

The workflow:
1. Checks out the repository
2. Sets up Node.js 18
3. Installs dependencies from `tools/mawesome/package.json`
4. Runs the generator with `PERSONAL_TOKEN` and `GITHUB_TOKEN` secrets
5. Commits and pushes if README.md changed

### Generator Tool (`tools/mawesome/index.js`)

- Uses `@octokit/rest` to fetch starred repositories via GitHub API
- Paginates through results (100 per page)
- Groups repos by language (via the template)
- Renders output using EJS template
- Uses `ejs.renderFile()` with `async: true` to support template includes and async operations

### EJS Template (`template/README.ejs`)

- Receives data object with `githubName` and `stars` array
- Each star has: `full_name`, `html_url`, `description`, `language`, `stargazers_count`, `owner`
- Iterates through language-grouped repos to generate categorized markdown

### Cloudflare Worker (`worker.js`)

- Fetches `data.json` from CDN (jsdelivr)
- Generates responsive HTML with Tailwind CSS
- Includes search functionality and sidebar navigation
- Displays repo cards with avatar, description, topics, and stats

## Environment Variables

- `PERSONAL_TOKEN` - GitHub personal access token (preferred for higher rate limits)
- `GITHUB_TOKEN` - GitHub Actions token (fallback, automatic in workflows)
- `USER_EMAIL` - Git commit email (optional, defaults to actor's noreply email)
- `GITHUB_REPOSITORY_OWNER` - Auto-set in workflows, used as default username
